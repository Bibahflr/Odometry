# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# %%
# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")

# %%
# 3. Simple Test: Post a message to CoppeliaSim status bar
sim.addLog(1, "Hello from Python!")
p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")

rw = 0.195/2
rb = 0.381/2
d = 0.05

#List untuk menampung data plot 
time_data = []
wr_list = []
wl_list = []
vx_list = []
w_list = []


# %%
try:
    # 4. Main Loop (Run for 40 seconds)
    start_time = time.time()
    print("recording data...")
    while (time.time() - start_time) < 40:
        
        # --- STUDENT CODE GOES HERE ---
        # Example: Print elapsed time
        elapsed = time.time() - start_time
        print(f"Running... {elapsed:.1f}s", end="\r")
        
        #time.sleep(0.1) 
        wr_vel = sim.getJointTargetVelocity(p3dx_RW)
        wl_vel = sim.getJointTargetVelocity(p3dx_LW)

        #Hitung kecepatan linear tiap roda (m/s)
        vr = wr_vel*rw
        vl = wl_vel*rw

        #Hitung kecepatan body
        vx = (wr_vel+ wl_vel)*rw/rb
        wx = (wr_vel- wl_vel)*rw/rb

        #Simpan data ke list
        time_data.append(elapsed)
        wr_list.append(wr_vel)
        wl_list.append(wl_vel)
        vx_list.append(vx)
        w_list.append(wx)

        sim.addLog(1, f"Vx:{vx:.2f}m/s, W:{wx:.2f}rad/s")
        print(f"Time: {elapsed:.1f}s | Vx: {vx:.2f}", end="\r")
        
        time.sleep(0.05)


finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped. Generating Plots...")

     # 6. Plotting
    plt.figure(figsize=(10, 8))

    # Plot Kecepatan Roda [cite: 131, 133]
    plt.subplot(2, 1, 1)
    plt.plot(time_data, wr_list, label='Right Wheel ($\dot{\\varphi}_R$)')
    plt.plot(time_data, wl_list, label='Left Wheel ($\dot{\\varphi}_L$)')
    plt.title('P3DX Joint Velocity')
    plt.ylabel('Velocity (rad/s)')
    plt.legend()
    plt.grid(True)

    # Plot Kecepatan Body [cite: 136, 137]
    plt.subplot(2, 1, 2)
    plt.plot(time_data, vx_list, label='Linear Velocity ($v_x$)')
    plt.plot(time_data, w_list, label='Angular Velocity ($\omega$)', color='orange')
    plt.title('P3DX Body Velocity')
    plt.xlabel('Time (sec)')
    plt.ylabel('Velocity (m/s or rad/s)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()