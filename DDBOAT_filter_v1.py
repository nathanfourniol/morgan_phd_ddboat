#!/usr/bin/env python3
################################################
# Auteurs : Morgan Louedec & BATOTOCLOCLOJOJO & tarpon
# Date : 25/02/2022
# Libre de droit mais citer les auteurs
################################################
# This file is a library to manage the measurements of the DDBOATs
#################################################
import numpy as np
from math import cos, sin, pi
import time



def sawtooth(x):
    return (x + pi) % (2 * pi) - pi  # or equivalently   2*arctan(tan(x/2))


def taking_a_measurement(imu):
    test = input()
    x1 = np.zeros((3, 1))
    if test == "y":
        x1 = np.array([imu.read_mag_raw()]).T  # [0]
    print("measuring ", x1.T)
    return x1
    
def delta_odo (odo1,odo0):
    dodo = odo1-odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


class DdboatFilter:
    def __init__(self, lxm, lym, A, b, encoddrv):
        sync, data_encoders = encoddrv.read_packet()
        self.odoLeft, self.odoRight = data_encoders[3], data_encoders[4]  # memorised position of the motors' encoder
        self.lxm, self.lym = lxm, lym,  # longitude and latitude of the origin
        self.rho = 6371009  # radius of the earth (m)
        self.A, self.b = A, b  # calibration parameters of the compass
        self.beta = 46000  # nT, norm of the magnetic field

    def measure_wm(self, data_encoders, t_boucle):  # measure the rational speed of the motors (turn/sec)
        # data_encoders : data from encod.read_packet()
        # t_boucle : period of the update (s)
        sensLeft, sensRight = data_encoders[1], data_encoders[2]
        odoLeft, odoRight = data_encoders[3], data_encoders[4]  # 0 to 65536=2**16 tics
        dodoLeft = abs(delta_odo(odoLeft,self.odoLeft)) # always positive
        dodoRight = abs(delta_odo(odoRight,self.odoRight)) 
        self.odoLeft, self.odoRight = odoLeft, odoRight
        wmLeft = dodoLeft / 8 / t_boucle  # 8 tics = 1 turn
        wmRight = dodoRight / 8 / t_boucle
        return wmLeft, wmRight

    @staticmethod
    def cvt_gll_ddmm_2_dd(val):  # get lat lon from gps raw data val
        ilat, ilon = val[0], val[2]
        olat = float(int(ilat / 100))
        olon = float(int(ilon / 100))
        olat_mm = (ilat % 100) / 60
        olon_mm = (ilon % 100) / 60
        olat += olat_mm
        olon += olon_mm
        if val[3] == "W":
            olon = -olon
        return olat, olon

    def latlon_to_coord(self, lat, lon):
        pos = np.array(
            [[self.rho * np.cos(lat * np.pi / 180) * (lon - self.lxm) * np.pi / 180],
             [self.rho * (lat - self.lym) * np.pi / 180]])
        return pos  # x in lon, y in lat

    def compass_calibration(self, imu):  # from student
        # imu : Imu9IO driver

        fichier = open("compass_calibration.txt", "w")
        n_satisfait = 1  # boolean flag to stop procedure

        # Boucle de demande de mesure à l'utilisateur
        while n_satisfait:
            print("Prêt mesure x1 ? (y/n)")
            x1 = taking_a_measurement(imu)

            print("Prêt mesure x-1 ? (y/n)")
            x_1 = taking_a_measurement(imu)

            print("Prêt mesure x2 ? (y/n)")
            x2 = taking_a_measurement(imu)

            print("Prêt mesure x3 ? (y/n)")
            x3 = taking_a_measurement(imu)

            print("Satisfait ? (y/n)")
            test_final = input()
            if test_final == "y":
                n_satisfait = 0

        fichier.close()

        # Calcul de calibration de la boussole, A et b
        try:
            # beta = 46000  # nT, norm of the magnetic field
            self.b = -(x1 + x_1) / 2
            X = np.hstack((x1 + self.b, x2 + self.b, x3 + self.b))
            self.A = (1 / self.beta) * X
        except:
            print("ERROR : missing some measurements !!!")

    def compass_auto_calibration(self, imu):  # when the magnetic field direction is unknown
        # mag_corrected = A_inv @ ( mag + b)

        # fichier = open("compass_calibration.txt", "w")
        # n_satisfait = 1  # boolean flag to stop procedure
        #
        # # step 1, taking 6 sample in specific positions
        # print("Compas Calibration : ")
        # print("m1 is Level")
        # print("m2 is nose up")
        # print("m3 is nose down")
        # print("m4 is nose left")
        # print("m5 is nose right")
        # print("m6 is flipped")
        # m_list = np.zeros((3,6))
        # while n_satisfait:
        #     print("Prêt mesure m1 ? (O/N)")
        #     m_list[:,[0]] = taking_a_measurement(imu,fichier)
        #
        #     print("Prêt mesure m2 ? (O/N)")
        #     m_list[:,[1]] = taking_a_measurement(imu,fichier)
        #
        #     print("Prêt mesure m3 ? (O/N)")
        #     m_list[:,[2]] = taking_a_measurement(imu,fichier)
        #
        #     print("Prêt mesure m4 ? (O/N)")
        #     m_list[:,[3]] = taking_a_measurement(imu,fichier)
        #
        #     print("Prêt mesure m5 ? (O/N)")
        #     m_list[:,[4]] = taking_a_measurement(imu, fichier)
        #
        #     print("Prêt mesure m6 ? (O/N)")
        #     m_list[:,[5]] = taking_a_measurement(imu, fichier)
        #
        #     print("Satisfait ? (O/N)")
        #     test_final = input()
        #     if test_final == "O":
        #         n_satisfait = 0
        #         fichier.write(str(m_list))
        m_list = np.array(
            [[-893, 972, 2637], [2254, 679, 619], [-2310, 1401, -847], [-2135, -1158, 2278], [1076, -584, 2423],
             [-418, -2157, -2582]]).T

        print("Compas Calibration : move the DDBOAT in all directions")
        dt_, N, mag_list = 0.5, 30, []
        for k in range(N):
            mag_list.append(np.array([imu.read_mag_raw()]).T)
            time.sleep(dt_)
        print("Measurement done, Calibrating")

        # step 2, find hard iron offset
        # b = np.zeros((3,1)) # = -1/N * sum(mag)
        # for mag in mag_list:
        #     b = b-mag
        # b = 1/N *b
        b = (m_list[:, [0]] - m_list[:, [1]] - m_list[:, [2]]
             - m_list[:, [3]] - m_list[:, [4]] - m_list[:, [5]]) / 4  # (m2+m3+m4+m5+m6-m1)/6

        # step 3, find soft iron matrix
        # z = m + b = A @ mc = beta * A @ e = Gamma_z @ e with e normalized and magnetic amplitude beta
        # z.T @ Q @ z = 1 , Q = Gamma_z_inv @ Gamma_z_inv found by LMS
        # A = Gamma_Q / beta

        M = np.zeros((N, 6))  # M@q = ones
        ones = np.ones((N, 1))
        for i in range(N):
            z = (mag_list[i] + b) / self.beta
            zx, zy, zz = z[0, 0], z[1, 0], z[2, 0]
            M[i] = [zx * zx, 2 * zx * zy, 2 * zx * zz, zy * zy, zy * zz, zz * zz]  # line i
        # M = np.zeros((6, 6))
        # ones = np.ones((6, 1))
        # for i in range(6):
        #     z = (m_list[:,[i]] + b)/self.beta
        #     print(z.T)
        #     zx, zy, zz = z[0,0], z[1,0], z[2,0]
        #     M[i] = [zx*zx, 2*zx*zy, 2*zx*zz, zy*zy, 2*zy*zz, zz*zz]# line i
        q = np.linalg.inv(M.T @ M) @ M.T @ ones  # pseudo inv
        Q = np.array([[q[0, 0], q[1, 0], q[2, 0]], [q[1, 0], q[3, 0], q[4, 0]], [q[2, 0], q[4, 0], q[5, 0]]])
        d, P = np.linalg.eig(Q)
        Gamma_Q = np.linalg.inv(P @ np.sqrt(np.diag(d)) @ np.linalg.inv(P))
        A = Gamma_Q
        print("Q is ", Q)
        print("eig Q is", d)
        print("A is ", A.flatten())
        print("b is ", b.T)
        print("Compass calibration done")
        return

    def cap(self, magx, magy, magz):  # is 0 when facing east, trigonometric sign
        # magx, magy, magz : magnetic vector from imu.read_mag_raw()
        x = np.array([[magx, magy, magz]]).T
        magx, magy, magz = np.linalg.inv(self.A) @ (x + self.b).flatten()  # correction of the magnetic field
        return sawtooth(-np.arctan2(magy, magx) + pi / 2)


# class StateObserver:
#     # Extended Kalman filter to estimate position and speed and the wind (qx,qy)
#     def __init__(self, X0: np.array, Gamma0: np.array, Gamma_alpha: np.array, Gamma_beta: np.array, dt: float):
#         self.X = X0  # initial state (x,y,v,th,qx,qy)
#         self.Gamma = Gamma0
#         self.Gamma_alpha = Gamma_alpha
#         self.Gamma_beta = Gamma_beta
#         self.dt = dt  # loop period
#         self.u = np.array([[0, 0]]).T
#         self.y = np.array([[0, 0, 0, 0]]).T
#
#     def p(self):  # robot position
#         return self.X[0:2, :]
#
#     def p_dot(self):  # robot speed vector
#         return np.array([[self.X[2, 0] * cos(self.X[3, 0]), self.X[2, 0] * sin(self.X[3, 0])]]).T
#
#     def Kalman_update(self, u):  # time update based on the movement of the robot
#         # u : [acceleration, angular velocity] control signal
#         self.u = u
#         th, v = self.X[3, 0], self.X[2, 0]
#         Ak = np.eye(6) + self.dt * np.array(
#             [[0, 0, cos(th), -v * sin(th),1,0], [0, 0, sin(th), v * cos(th),0,1],
#              [0, 0, 0, 0,0,0], [0, 0, 0, 0,0,0],[0, 0, 0, 0,0,0], [0, 0, 0, 0,0,0]])
#         self.X = self.X + self.dt * np.array([[v * cos(th)], [v * sin(th)], [u[0, 0]], [0],[0],[0]]) # no th update
#         self.Gamma = Ak @ self.Gamma @ Ak.T + self.Gamma_alpha
#         return
#
#     def Kalman_correct(self, y):  # measurement correction of the kalman filter
#         # y : measure [x,y,y_v**2,y_th]
#         self.y = y
#         Ck = np.array([[1, 0, 0, 0,0,0], [0, 1, 0, 0,0,0], [0, 0, 0, 0,0,0], [0, 0, 0, 1,0,0]])
#         Ck[3,2] = 2*(self.X[2,0]+self.X[4,0]*cos(self.X[3,0])+self.X[5,0]*sin(self.X[3,0]))
#         Ck[3,3] = 2*self.X[2,0]*(self.X[5,0]*cos(self.X[3,0])-self.X[4,0]*sin(self.X[3,0]))
#         Ck[3,4] = 2*(self.X[2,0]*cos(self.X[3,0])+self.X[4,0])
#         Ck[3, 5] = 2 * (self.X[2, 0] * sin(self.X[3, 0]) + self.X[5, 0])
#         Zk = y - Ck @ self.X
#         Zk[3,0] = sawtooth(Zk[3,0]) # special correction for th
#         Sk = Ck @ self.Gamma @ Ck.T + self.Gamma_beta
#         Kk = self.Gamma @ Ck.T @ np.linalg.inv(Sk)
#         self.X = self.X + Kk @ Zk
#         self.X[3,0] = sawtooth(self.X[3,0])
#         self.Gamma = (np.eye(6) - Kk @ Ck) @ self.Gamma
#         return
#
#     def Kalman_correct_heading_only(self, y):  # can only correct heading when no gps data
#         # y : measure [th]
#         self.y[3,0] = y[0,0]
#         Ck = np.array([[0, 0, 0, 1,0,0]])
#         Zk = y - Ck @ self.X
#         Zk[0,0] = sawtooth(Zk[0,0]) # special correction for th
#         Sk = Ck @ self.Gamma @ Ck.T + self.Gamma_beta[3,3]
#         Kk = self.Gamma @ Ck.T @ np.linalg.inv(Sk)
#         self.X = self.X + Kk @ Zk
#         self.X[3,0] = sawtooth(self.X[3,0])
#         self.Gamma = (np.eye(6) - Kk @ Ck) @ self.Gamma
#         return

class StateObserver:
    # Extended Kalman filter to estimate position and speed
    def __init__(self, X0: np.array, th0: float, Gamma0: np.array, Gamma_alpha: np.array, Gamma_beta: np.array,
                 dt: float):
        self.X = X0  # initial state (x,y,v,qx,qy)
        self.Gamma = Gamma0
        self.Gamma_alpha = Gamma_alpha
        self.Gamma_beta = Gamma_beta
        self.dt = dt  # loop period
        self.u = np.array([[0, 0]]).T
        self.y = np.array([[0, 0]]).T
        self.th = th0  # measurement of the heading
        self.Ck = np.array([[1, 0, 0, 0,0], [0, 1, 0, 0,0]])

    def p(self):  # robot position
        return self.X[0:2, :]

    def p_dot(self):  # robot speed vector
        return np.array([[self.X[2, 0] * cos(self.th) + self.X[3, 0], self.X[2, 0] * sin(self.th) + self.X[4, 0]]]).T

    def Kalman_update(self, u,th):  # time update based on the movement of the robot
        # u : [acceleration, angular velocity] control signal
        self.u,self.th = u,th
        Ak = np.eye(5) + self.dt * np.array(
            [[0, 0, cos(self.th), 1, 0],
             [0, 0, sin(self.th), 0, 1],
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0,0],
             [0, 0, 0, 0, 0]])
        self.X = self.X + self.dt * np.array([[self.X[2,0] * cos(self.th)+self.X[3,0]],
                                              [self.X[2,0] * sin(self.th)+self.X[4,0]],
                                              [u[0, 0]],
                                              [0],
                                              [0]])
        self.Gamma = Ak @ self.Gamma @ Ak.T + self.Gamma_alpha
        return

    def Kalman_correct(self, y):  # measurement correction of the kalman filter
        # y : measure [x,y,x-x_old,y-y_old]
        self.y = y
        Zk = y - self.Ck @ self.X # g(x) is linear
        Sk = self.Ck @ self.Gamma @ self.Ck.T + self.Gamma_beta
        Kk = self.Gamma @ self.Ck.T @ np.linalg.inv(Sk)
        self.X = self.X + Kk @ Zk
        self.Gamma = (np.eye(5) - Kk @ self.Ck) @ self.Gamma
        return
