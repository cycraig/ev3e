#!/usr/bin/python
import numpy as np

# Globals
timeout = 0.2
burst_rate = 20
rate_const = 25

# Yaw constants
Kp_yaw = 0.2
Ki_yaw = 0.003 # 0.001
Kd_yaw = 0.0003 # 0.0001

# xyz constants
Kp = 0.5
Ki = 0.001 # 0.0002
Kd = 0.0001 # 0.00002

def pid_yaw(Perr_old, Perr, Pierr, Piderr, roll, pitch, yaw):
    target = 0.0 # assumed to reset the yaw rate

    Perr = target - yaw
    Pierr = Pierr + (target - yaw)
    Piderr = Perr - Perr_old
  
    u = Kp_yaw*Perr + Ki_yaw*Pierr + Kd_yaw*Piderr

    return Perr,Pierr,Piderr,u

def pid_xyz(Perr_old, Perr, Pierr, Piderr, x, y, z, target_x, target_y, target_z):
    drone = np.array([x, y, z])
    target = np.array([target_x, target_y, target_z])

    Perr = target - drone
    Pierr = Pierr + (target - drone)
    # Piderr = Piderr - (target - drone)
    Piderr = Perr - Perr_old

    u = Kp*Perr + Ki*Pierr + Kd*Piderr

    return Perr,Pierr,Piderr,u