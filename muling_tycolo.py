# main script of the anger mission, the robot must follow a desired timed trajectory

from DDBOAT_controler_v1 import *
from DDBOAT_filter_v1 import *
from log_driver import LogRecorder, init_drivers, time
import json


def reached(position, objective, precision):
    ''' Check if position in near objective with the precision'''
    xmin, xmax = objective[0] - precision[0], objective[0] + precision[0]
    ymin, ymax = objective[1] - precision[1], objective[1] + precision[1]
    if (position[0] > xmin and position[0] < xmax) and (position[1] > ymin and position[1] < ymax):
        # position is in the square (xmin, ymax), (xmin, ymin), (xmax, ymax), (xmax, ymin)
        return True
    else:
        return False

#######################
# robot setup
#######################

print("robot setup ...")
time_mission_max = 100
# Robot set-up
mission_robot_id = 1 
zone = ((0, 0), (17, 10)) # en coord locale

file_script2 = open("compass_calibration/compass_calibration_ddboat8.json", "r")
data_script2 = json.load(file_script2)
print("robot id", mission_robot_id)

ard, temperature, gps, encoddrv, imu = init_drivers()
log_rec = LogRecorder()
lxm = -4.616068   # point de reference
lym = 48.43127   # point de reference

b = np.reshape(np.array([data_script2["b"]]), (3, 1))
A = np.reshape(np.array([data_script2["A"]]), (3, 3))

filt = DdboatFilter(lxm, lym, A, b, encoddrv)

Gamma0 = np.diag([10, 10, 1, 10, 10])
Gamma_alpha = np.diag([0.5, 0.5, 0.1, 0.0001, 0.0001])
Gamma_beta = np.diag([0.1, 0.1])
dt = 0.1  # 10hz


while True:  # find initial pose
    _, _, _, gll_ok, val, _, _, mag, _, _ = log_rec.log_observe_update(temperature, ard, gps, encoddrv, imu)
    log_rec.log_update_write()
    if gll_ok:
        y_th = filt.cap(mag[0], mag[1], mag[2])
        lat, lon = filt.cvt_gll_ddmm_2_dd(val)
        # print("LAT LON ", lat, lon)
        pos = filt.latlon_to_coord(lat, lon)
        # print("POSE : ", pos)
        X0 = np.array([[pos[0, 0], pos[1, 0], 1, 0, 0]]).T  # note: initial speed set to 1 to avoid singularity
        break
    time.sleep(0.1)

print("Initial state is", X0.T)
kal = StateObserver(X0, y_th, Gamma0, Gamma_alpha, Gamma_beta, dt)
print("Robot setup done")

#####################
# Go to init place
#####################
init_ok = True
k, pd, pos_old, t_pos_old, cmdL, cmdR = 0, None, kal.p(), time.time(), 0, 0

home_lat, home_lon = 48.431273, -4.615776
UP = filt.latlon_to_coord(home_lat, home_lon)
obj_lat, obj_lon = 48.431152, -4.615430
DOWN = filt.latlon_to_coord(obj_lat, obj_lon)

print("OBJECTIF = UP")
OBJECTIF = UP
print("OBJECTIF =", UP)
BOOL_OBJ = True

t0 = time.time()
t_chg_obj = time.time()

while init_ok:
    t = time.time()
    tl, tr, data, gll_ok, val, sync, data_encoders, mag, accel, gyro = log_rec.log_observe_update(temperature, ard, gps, encoddrv, imu)
    wmLeft, wmRight = filt.measure_wm(data_encoders, dt)
    y_th = filt.cap(mag[0], mag[1], mag[2])
    if gll_ok:  # correct kalman filter with new gps data
        lat, lon = filt.cvt_gll_ddmm_2_dd(val)
        pos = filt.latlon_to_coord(lat, lon)
        kal.Kalman_correct(np.array([[pos[0, 0], pos[1, 0]]]).T)
    
    mstop = 0.85

    # controler update
    pd_dot, pd_ddot = np.zeros((2,1)), np.zeros((2,1))
    u = mstop*control_feedback_linearization(OBJECTIF, pd_dot, pd_ddot, dt, p=kal.p(), v=kal.X[2, 0], th=y_th, qx=0*kal.X[3, 0], qy=0*kal.X[4, 0])

    cmdL, cmdR = convert_motor_control_signal(u, kal.X[2, 0], wmLeft, wmRight, cmdL, cmdR, dt)
    cmdL, cmdR = mstop*cmdL, mstop*cmdR
    print("CMDLOG L R ", cmdL, cmdR)
    ard.send_arduino_cmd_motor(cmdL, cmdR)
    log_rec.log_control_update(u[0, 0], u[1, 0], wmLeft, wmRight, cmdL, cmdR, pd, y_th, kal)
    kal.Kalman_update(u, y_th)
    log_rec.log_update_write()  # write in the log file
    
    if reached((kal.X[0], kal.X[1]), (OBJECTIF[0][0], OBJECTIF[1][0]), (1, 1)):
        init_ok = True  # mule is at its right bound
        print("REACHED")
        break

    # loop update
    if not sync:
        print("arduino communication lost, break !")
        break
    print("TEXEC ", time.time()-t0)
    if (time.time()-t0) > time_mission_max:
        print("maximum allowed time passed, breaking !")
        break
    if (time.time()-t_chg_obj) > 30 :
        if BOOL_OBJ:
            print("OBJECTIF = DOWN")
            OBJECTIF = DOWN
            BOOL_OBJ = False
        else:
            print("OBJECTIF = UP")
            OBJECTIF = UP
            BOOL_OBJ = True
        t_chg_obj = time.time()

    t_execution = time.time() - t
    delta_t = dt - t_execution
    if delta_t > 0:
      time.sleep(delta_t)
    else:
        print("LAG loop frequency reduced, t_execution ", t_execution)

ard.send_arduino_cmd_motor(0, 0)
