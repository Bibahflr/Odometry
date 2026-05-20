# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# =========================================================
# CONNECT TO COPPELIASIM
# =========================================================
client = RemoteAPIClient()
sim = client.require('sim')

sim.startSimulation()
print("Simulation Started")

# =========================================================
# TRANSFORMATION MATRIX
# =========================================================
def transformMat(alpha, beta, gamma, tx, ty, tz):

    rotx = np.array([
        [1,0,0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha),  math.cos(alpha)]
    ])

    roty = np.array([
        [ math.cos(beta),0,math.sin(beta)],
        [0,1,0],
        [-math.sin(beta),0,math.cos(beta)]
    ])

    rotz = np.array([
        [math.cos(gamma), -math.sin(gamma),0],
        [math.sin(gamma),  math.cos(gamma),0],
        [0,0,1]
    ])

    rot_total = rotx @ roty @ rotz

    trans_vector = np.array([
        [tx],
        [ty],
        [tz]
    ])

    Rt = np.hstack((rot_total, trans_vector))

    homo = np.array([[0,0,0,1]])

    T = np.vstack((Rt, homo))

    return T

# =========================================================
# ROBOT HANDLE
# =========================================================
p3dx = sim.getObject('/PioneerP3DX')

rightMotor = sim.getObject('/PioneerP3DX/rightMotor')
leftMotor  = sim.getObject('/PioneerP3DX/leftMotor')

# =========================================================
# SENSOR HANDLE (4 SENSOR)
# =========================================================
sensorHandles = []

sensor_index = [0,3,4,7]

for i in sensor_index:

    sensorHandles.append(
        sim.getObject(
            f'/PioneerP3DX/ultrasonicSensor[{i}]'
        )
    )

# =========================================================
# WAYPOINT HANDLE
# =========================================================
path_Handle = []

TOTAL_WAYPOINT = 56

# cylinder pertama
path_Handle.append(
    sim.getObject('/Cylinder')
)

# cylinder sisanya
for i in range(56):

    path_Handle.append(
        sim.getObject(f'/Cylinder[{i}]')
    )

# =========================================================
# VISUAL HANDLE
# =========================================================
LH_Handle = sim.getObject('/LH')
Perp_Handle = sim.getObject('/Perp')

# =========================================================
# ROBOT PARAMETER
# =========================================================
rw = 0.195/2
rb = 0.318

# FIX PENTING
LH_distance = 0.4

# =========================================================
# OGM
# =========================================================
map_size = 250
resolution = 0.05

ogm = np.zeros((map_size,map_size))

map_center = map_size // 2

# =========================================================
# WAYPOINT INDEX
# =========================================================
current_waypoint = 0

# =========================================================
# MAIN LOOP
# =========================================================
try:

    start_time = time.time()

    while(time.time() - start_time < 120):

        # =================================================
        # ROBOT POSE
        # =================================================
        robot_pos = sim.getObjectPosition(
            p3dx,
            sim.handle_world
        )

        robot_ori = sim.getObjectOrientation(
            p3dx,
            sim.handle_world
        )

        # =================================================
        # LOOK AHEAD POINT
        # =================================================
        LH_world = transformMat(
            robot_ori[0],
            robot_ori[1],
            robot_ori[2],
            robot_pos[0],
            robot_pos[1],
            robot_pos[2]
        ) @ np.array([
            [LH_distance],
            [0],
            [0],
            [1]
        ])

        LH_world = LH_world[:3,:]

        # =================================================
        # CURRENT TARGET WAYPOINT
        # =================================================
        target_pos = sim.getObjectPosition(
            path_Handle[current_waypoint],
            sim.handle_world
        )

        # =================================================
        # DISTANCE LH TO TARGET
        # =================================================
        dist_target = math.sqrt(
            (LH_world[0,0] - target_pos[0])**2 +
            (LH_world[1,0] - target_pos[1])**2
        )

        # =================================================
        # NEXT WAYPOINT
        # =================================================
        if dist_target < 0.35:

            current_waypoint += 1

            if current_waypoint >= TOTAL_WAYPOINT:

                current_waypoint = 0

        # =================================================
        # WORLD TO ROBOT FRAME
        # =================================================
        T_world_robot = transformMat(
            0,
            0,
            robot_ori[2],
            robot_pos[0],
            robot_pos[1],
            robot_pos[2]
        )

        desired_robot = np.linalg.inv(
            T_world_robot
        ) @ np.array([
            [target_pos[0]],
            [target_pos[1]],
            [target_pos[2]],
            [1]
        ])

        # =================================================
        # TRACKING ERROR
        # =================================================
        ed = math.sqrt(
            desired_robot[0,0]**2 +
            desired_robot[1,0]**2
        )

        eh = math.atan2(
            desired_robot[1,0],
            desired_robot[0,0]
        )

        # =================================================
        # CONTROLLER (LEBIH STABIL)
        # =================================================
        vx = 0.8 * ed
        wx = 1.8 * eh

        # limit speed
        if vx > 1.0:
            vx = 1.0

        # =================================================
        # WHEEL SPEED
        # =================================================
        wr = (vx + (rb*wx)/2)/rw
        wl = (vx - (rb*wx)/2)/rw

        sim.setJointTargetVelocity(
            rightMotor,
            wr
        )

        sim.setJointTargetVelocity(
            leftMotor,
            wl
        )

        # =================================================
        # SENSOR MAPPING
        # =================================================
        for sensor in sensorHandles:

            result, distance, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.readProximitySensor(sensor)

            if result:

                sensor_pos = sim.getObjectPosition(
                    sensor,
                    sim.handle_world
                )

                sensor_ori = sim.getObjectOrientation(
                    sensor,
                    sim.handle_world
                )

                T_world_sensor = transformMat(
                    sensor_ori[0],
                    sensor_ori[1],
                    sensor_ori[2],
                    sensor_pos[0],
                    sensor_pos[1],
                    sensor_pos[2]
                )

                detected_local = np.array([
                    [detectedPoint[0]],
                    [detectedPoint[1]],
                    [detectedPoint[2]],
                    [1]
                ])

                detected_world = T_world_sensor @ detected_local

                obstacle_x = detected_world[0,0]
                obstacle_y = detected_world[1,0]

                grid_x = int(
                    obstacle_x/resolution
                ) + map_center

                grid_y = int(
                    obstacle_y/resolution
                ) + map_center

                if (
                    0 <= grid_x < map_size
                    and
                    0 <= grid_y < map_size
                ):

                    ogm[grid_y,grid_x] = 1

        # =================================================
        # VISUALIZATION
        # =================================================
        sim.setObjectPosition(
            LH_Handle,
            sim.handle_world,
            LH_world.flatten().tolist()
        )

        sim.setObjectPosition(
            Perp_Handle,
            sim.handle_world,
            target_pos
        )

        time.sleep(0.01)

# =========================================================
# STOP
# =========================================================
finally:

    sim.stopSimulation()

    print("Simulation Stopped")

# =========================================================
# SHOW OGM
# =========================================================
plt.figure(figsize=(8,8))

plt.imshow(
    ogm,
    cmap='gray',
    origin='lower'
)

plt.title("Occupancy Grid Map")

plt.xlabel("X")
plt.ylabel("Y")

plt.show()

# %%