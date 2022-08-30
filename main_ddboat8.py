# main script of the anger mission, the robot must follow a desired timed trajectory
import numpy as np

from DDBOAT_controler_v1 import *
from DDBOAT_filter_v1 import *
from log_driver import LogRecorder, init_drivers, time, robot_number
import json
import sys
import data_muling.ddboat.dubins_car_angers as mule

#######################
# robot setup
#######################

print("robot setup ...")

# load mission script
file_script = open("muling_parameters.json", "r")
data_script = json.load(file_script)
param = data_script["mission_param"]

file_script2 = open("compass_calibration/compass_calibration_ddboat"+robot_number+".json", "r")
data_script2 = json.load(file_script2)
print("robot id", 1)

try:
    hour = str(sys.argv[1])
except:
    hour = "0"
try:
    minute = str(sys.argv[2])
except:
    minute = "0"
t = time.localtime(time.time())
dtStr = str(t.tm_year) + "-" + str(t.tm_mon) + "-" + str(t.tm_mday) + "-" + hour + "-" + minute + "-" + "0"
local_time_mission_begin = time.strptime(dtStr, "%Y-%m-%d-%H-%M-%S")
time_mission_begin = time.mktime(local_time_mission_begin)

ard, temperature, gps, encoddrv, imu = init_drivers()
log_rec = LogRecorder(local_time_mission_begin)
lym = param["home_lat"]
lxm = param["home_lon"]
b = np.reshape(np.array([data_script2["b"]]), (3, 1))
A = np.reshape(np.array([data_script2["A"]]), (3, 3))

filt = DdboatFilter(lxm, lym, A, b, encoddrv)

Gamma0 = np.diag(param["Gamma0"])
Gamma_alpha = np.diag(param["Gamma_alpha"])
Gamma_beta = np.diag(param["Gamma_beta"])

# Init mule
d = param["d"]
verbose_m = False
verbose_s = 1
borne_inf_latlon = (lym, lxm)
borne_inf_local = filt.latlon_to_coord(borne_inf_latlon[0], borne_inf_latlon[1])
borne_sup_latlon = (48.430855, -4.615595)
borne_sup_local = filt.latlon_to_coord(borne_sup_latlon[0], borne_sup_latlon[1])
bornes = (borne_inf_local, borne_sup_local)
print("COORD BORNES : ", bornes) 
m = mule.Dubins(1, 0, 0, 0, 0, bornes, d, "10.42.0.208", "10.42.0.1", verbose_m, verbose_s)
print("INIT MULE : ", m.state)

while True:  # find initial pose
    _, _, _, gll_ok, val, _, _, mag, _, _ = log_rec.log_observe_update(temperature, ard, gps, encoddrv, imu)
    log_rec.log_update_write()
    if gll_ok:
        y_th = filt.cap(mag[0], mag[1], mag[2])
        lat, lon = filt.cvt_gll_ddmm_2_dd(val)
        pos = filt.latlon_to_coord(lat, lon)
        X0 = np.array([[pos[0, 0], pos[1, 0], 1, 0, 0]]).T  # note: initial speed set to 1 to avoid singularity
        break
    time.sleep(0.1)

dt = param["dt"]  # 10hz
# ~ print("initial state is", X0.T)
kal = StateObserver(X0, y_th, Gamma0, Gamma_alpha, Gamma_beta, dt)
print("robot setup done")
print("---")

time_mission_max = param["duration_mission_max"] + time.time()  # max allowed time for mission
# print("mission will begin at", time.asctime(local_time_mission_begin))

return_home = True # if true, at the en of the mission, the robot return home
home_lat, home_lon = param["home_lat"], param["home_lon"]
home_pos = filt.latlon_to_coord(home_lat, home_lon)

#####################
# mission loop
#####################
mission = True
k, pd, pos_old, t_pos_old, cmdL, cmdR = 0, None, kal.p(), time.time(), 0, 0

t_adv = time.time()
p_motor = 1
while mission:

    # measurements
    t = time.time()
    tl, tr, data, gll_ok, val, sync, data_encoders, mag, accel, gyro = log_rec.log_observe_update(temperature, ard, gps,
                                                                                                  encoddrv, imu)
    wmLeft, wmRight = filt.measure_wm(data_encoders, dt)
    y_th = filt.cap(mag[0], mag[1], mag[2])
    if gll_ok:  # correct kalman filter with new gps data
        lat, lon = filt.cvt_gll_ddmm_2_dd(val)
        pos = filt.latlon_to_coord(lat, lon)
        kal.Kalman_correct(np.array([[pos[0, 0], pos[1, 0]]]).T)

    m.state_update(kal.p()[0], kal.p()[1], kal.X[2, 0], kal.th)
    OBJECTIF = m.run1step(t, dt)

    delta_adv = time.time()-t_adv 
    if delta_adv > 2:
        t_adv = time.time()
        # print('OBJ :', OBJECTIF) 
        print('POS :', m.state)
        print('CMD : ', cmdL, cmdR)

    # control update
    pd_dot, pd_ddot = np.zeros((2, 1)), np.zeros((2, 1))
    u = p_motor * control_feedback_linearization(OBJECTIF, pd_dot, pd_ddot, dt, p=kal.p(), v=kal.X[2, 0], th=y_th, qx=0*kal.X[3, 0], qy=0*kal.X[4, 0])
    cmdL, cmdR = convert_motor_control_signal(u, kal.X[2, 0], wmLeft, wmRight, cmdL, cmdR, dt)
    ard.send_arduino_cmd_motor(cmdL, cmdR)
    log_rec.log_control_update(u[0, 0], u[1, 0], wmLeft, wmRight, cmdL, cmdR, pd, y_th, kal)
    kal.Kalman_update(u, y_th)
    log_rec.log_update_write()  # write in the log file

    # loop update
    if not sync:
        print("arduino communication lost, break !")
        break
    if time.time() > time_mission_max:
        print("maximum allowed time passed, breaking !")
        break

    t_execution = time.time() - t
    delta_t = dt - t_execution
    if delta_t > 0:
        time.sleep(delta_t)
    else:
        print("LAG loop frequency reduced, t_execution ", t_execution)

ard.send_arduino_cmd_motor(0, 0)
