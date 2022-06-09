# main script of the anger mission, the robot must follow a desired timed trajectory

from DDBOAT_controler_v1 import *
from DDBOAT_filter_v1 import *
from log_driver import LogRecorder, init_drivers, time
import json
import sys

#######################
# robot setup
#######################

print("robot setup ...")

# load mission script
file_script = open("angers_mission_script.json", "r")
file_script2 = open("compass_calibration.json", "r")
data_script = json.load(file_script)
data_script2 = json.load(file_script2)
param = data_script["mission_param"]
trajs = data_script["trajectories"]
mission_robot_id = param["mission_robot_id"]
traj = trajs[mission_robot_id]
print("robot id", mission_robot_id)

try:
    hour = str(sys.argv[1])
except:
    hour = 0
try:
    minute = str(sys.argv[2])
except:
    minute = 0
tb = param["time_mission_begin"]
t = time.localtime(time.time())
dtStr = str(t.tm_year) + "-" + str(t.tm_mon) + "-" + str(t.tm_mday) + "-" + hour + "-" + minute + "-" + str(0)
local_time_mission_begin = time.strptime(dtStr, "%Y-%m-%d-%H-%M-%S")
time_mission_begin = time.mktime(local_time_mission_begin)

ard, temperature, gps, encoddrv, imu = init_drivers()
log_rec = LogRecorder(local_time_mission_begin)
lxm = param["lxm"]
lym = param["lym"]
b = np.reshape(np.array([data_script2["b"]]), (3, 1))
A = np.reshape(np.array([data_script2["A"]]), (3, 3))

filt = DdboatFilter(lxm, lym, A, b, encoddrv)

Gamma0 = np.diag(param["Gamma0"])
Gamma_alpha = np.diag(param["Gamma_alpha"])
Gamma_beta = np.diag(param["Gamma_beta"])
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
print("initial state is", X0.T)
kal = StateObserver(X0, y_th, Gamma0, Gamma_alpha, Gamma_beta, dt)
print("robot setup done")

#####################
# wait for mission beginning
#####################
time_mission_max = param["duration_mission_max"] + time.time()  # max allowed time for mission
print("mission will begin at", local_time_mission_begin)

#####################
# mission loop
#####################
mission = True
k, pd, pos_old, t_pos_old, cmdL, cmdR = 0, None, kal.p(), time.time(), 0, 0

print("Going to the initial waypoint")
wait_for_signal = True  # after time mission begin, the robot start following the trajectory
mstop = 1
CB = ControlBlock(dt, traj[0], r=4)

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
    CB.variable_update(p=kal.p(), v=kal.X[2, 0], th=y_th, qx=0 * kal.X[3, 0], qy=0 * kal.X[4, 0],
                       wmLeft=wmLeft, wmRight=wmRight, cmdL_old=cmdL, cmdR_old=cmdR)

    # control update
    if wait_for_signal:  # stand by initial waypoint
        cmdL, cmdR, u = CB.station_keeping1()
        if time.time() > time_mission_begin:  # after time mission begin, the robot start following the trajectory
            wait_for_signal = False
            print("Start following the trajectory")
    else:  # follow reference

        # update reference
        try:
            traj_k = traj[k]
            pd = np.reshape(np.array([traj_k["pd"]]), (2, 1))
            pd_dot = np.reshape(np.array([traj_k["pd_dot"]]), (2, 1))
            pd_ddot = np.reshape(np.array([traj_k["pd_ddot"]]), (2, 1))
            k += 1
        except:  # end of the trajectory
            print("end of the trajectory, break !")
            break

        cmdL, cmdR, u = CB.follow_reference(pd, pd_dot, pd_ddot)
    # ~ ard.send_arduino_cmd_motor(cmdL, cmdR)
    log_rec.log_control_update(u[0, 0], u[1, 0], wmLeft, wmRight, cmdL, cmdR, pd, y_th, kal)
    kal.Kalman_update(0 * u, y_th)
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
