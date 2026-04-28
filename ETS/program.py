from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import time

# ==========================================
# CONNECT
# ==========================================
client = RemoteAPIClient()
sim = client.require('sim')

if sim.getSimulationState() == 0:
    sim.startSimulation()
    time.sleep(1)

# ==========================================
# OBJECTS
# ==========================================
player = sim.getObject('/Robot_Pemain')
gk     = sim.getObject('/Robot_Lawan_01')
def1   = sim.getObject('/Robot_Lawan_02')
ball   = sim.getObject('/Bola_Merah')

l1 = sim.getObject('/Robot_Pemain/leftMotor')
r1 = sim.getObject('/Robot_Pemain/rightMotor')

l2 = sim.getObject('/Robot_Lawan_01/leftMotor')
r2 = sim.getObject('/Robot_Lawan_01/rightMotor')

l3 = sim.getObject('/Robot_Lawan_02/leftMotor')
r3 = sim.getObject('/Robot_Lawan_02/rightMotor')

# ==========================================
# GOAL
# ==========================================
GOAL_X = 5.125
GOAL_Y = -1.10

# ==========================================
# HELPERS
# ==========================================
def pos(obj):
    p = sim.getObjectPosition(obj, -1)
    return p[0], p[1]

def yaw(obj):
    return sim.getObjectOrientation(obj, -1)[2]

def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def motor(l, r, vl, vr):
    sim.setJointTargetVelocity(l, vl)
    sim.setJointTargetVelocity(r, vr)

def goto(robot, l, r, tx, ty, speed=4.5):
    x, y = pos(robot)
    th = yaw(robot)

    target = math.atan2(ty - y, tx - x)
    err = wrap(target - th)

    turn = 3.5 * err * 0.8

    vl = speed - turn
    vr = speed + turn

    vl = max(min(vl, 8), -8)
    vr = max(min(vr, 8), -8)

    motor(l, r, vl, vr)

def stop_all():
    motor(l1, r1, 0, 0)
    motor(l2, r2, 0, 0)
    motor(l3, r3, 0, 0)

# ==========================================
# MAIN LOOP
# ==========================================
try:
    while True:

        if sim.getSimulationState() == 0:
            time.sleep(0.1)
            continue

        px, py = pos(player)
        bx, by = pos(ball)
        gx, gy = pos(gk)

        # ==========================================
        # DISTANCES
        # ==========================================
        d_ball = math.sqrt((bx - px)**2 + (by - py)**2)
        d_goal = math.sqrt((GOAL_X - px)**2 + (GOAL_Y - py)**2)

        # ==========================================
        # GK AVOID (simple & stable)
        # ==========================================
        gk_dx = px - gx
        gk_dy = py - gy
        gk_dist = math.sqrt(gk_dx**2 + gk_dy**2)

        avoid_x = 0
        avoid_y = 0

        if gk_dist < 1.5:
            avoid_x = gk_dx / (gk_dist + 0.001)
            avoid_y = gk_dy / (gk_dist + 0.001)

        # ==========================================
        # GOAL DIRECTION (CLEAN)
        # ==========================================
        goal_x = GOAL_X - bx
        goal_y = GOAL_Y - by

        mag = math.sqrt(goal_x**2 + goal_y**2)
        if mag == 0:
            mag = 1

        goal_x /= mag
        goal_y /= mag

        dribble_dist = 0.25

        tx = bx - goal_x * dribble_dist + avoid_x * 0.4
        ty = by - goal_y * dribble_dist + avoid_y * 0.4

        # ==========================================
        # STRIKER LOGIC
        # ==========================================
        if d_ball > 0.32:

            speed = 3 + min(d_ball * 4, 5)
            speed = min(speed, 5.5)

            goto(player, l1, r1, tx, ty, speed)

        elif d_goal > 0.6:

            goto(player, l1, r1, GOAL_X, GOAL_Y, 3.8)

        else:

            shoot_y = GOAL_Y + (0.25 if by > GOAL_Y else -0.25)
            goto(player, l1, r1, GOAL_X, shoot_y, 6)

        # ==========================================
        # GK
        # ==========================================
        gk_x = 4.75
        gk_y = max(min(by, 0.9), -2.1)
        goto(gk, l2, r2, gk_x, gk_y, 2.5)

        # ==========================================
        # DEFENDER
        # ==========================================
        if bx > 0:
            dx = bx - 1.2
        else:
            dx = 1.5

        dy = max(min(by, 2.5), -2.5)
        goto(def1, l3, r3, dx, dy, 3)

        time.sleep(0.05)

except KeyboardInterrupt:
    pass

stop_all()
print("STOPPED")