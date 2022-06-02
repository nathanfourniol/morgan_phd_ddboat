# test the control of the motor in speed

from DDBOAT_filter_v1 import *
from log_driver import init_drivers, time


print("testing motor control")

ard, _, _, encoddrv, _ = init_drivers()
kpwm = 2
wmLeft_d, wmRight_d = 75, 50 # reference

filt = DdboatFilter(0, 0, np.zeros((3,3)), np.zeros((3,1)), encoddrv)

dt = 0.1  # 10hz
cmdL_old,cmdR_old = 0, 0
while True:

    # measurements
    t = time.time()
    try:
        sync, data_encoders = encoddrv.read_packet()
    except:
        sync = False
        data_encoders = []
        print("can't log the encoders")

    wmLeft, wmRight = filt.measure_wm(data_encoders, dt)
    print(" --- ")
    print("wmLeft = "+str(wmLeft)+" | wmRight = "+str(wmRight))
    print("data_encodeur ",data_encoders)

    cmdL = min(max(cmdL_old + dt * kpwm * (wmLeft_d - wmLeft), 0), 200)
    cmdR = min(max(cmdR_old + dt * kpwm * (wmRight_d - wmRight), 0), 200)
    print("cmdL = "+str(cmdL)+" | cmdR = "+str(cmdR))
    ard.send_arduino_cmd_motor(cmdL, cmdR)

    # loop update
    if not sync:
        print("arduino communication lost, break !")
        break

    t_execution = time.time() - t
    delta_t = dt - t_execution
    if delta_t > 0:
        time.sleep(delta_t)
    else:
        print("LAG loop frequency reduced, t_execution ", t_execution)

ard.send_arduino_cmd_motor(0, 0)
