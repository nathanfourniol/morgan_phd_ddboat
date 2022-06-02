# To only calibrate the compass

from DDBOAT_filter_v1 import *
from log_driver import LogRecorder, init_drivers,time

import json

# load mission script
file_script = open("angers_mission_script.json","r")
data_script = json.load(file_script)
param = data_script["mission_param"]

lxm = param["lxm"]
lym = param["lym"]
b = np.reshape(np.array([param["b"]]),(3,1))
A = np.reshape(np.array([param["A"]]),(3,3))

_, _, _, encoddrv, imu = init_drivers()
filt = DDBOAT_filter(lxm, lym, A, b,encoddrv)

filt.compass_auto_calibration(imu)