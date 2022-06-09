#!/usr/bin/env python3
################################################
# Auteurs : Morgan Louedec & BATOTOCLOCLOJOJO & tarpon
# Date : 23/02/2022
# Libre de droit mais citer les auteurs
################################################
# This file is a library for the controllers of the DDBoat
# current controllers:
# speed and heading / regul(v_d, th_d, th)
# point following / follow_point(b,m)
# line following / follow_line(a, b, m)
# station_keeping / control_station_keeping(a, d, pos, th, v)
# Lissajou curve / control_lisssajou(...)

# these function can be used as a controler bloc
# inputs : local positions (m), heading(rad), time(s), references
# output : left and right thruster speed (m/s)
#################################################

import numpy as np
from math import sin, cos, atan2

k1, k3 = 0.01,0.006#150, 50
K = np.array([[k1, k1], [-k3, k3]])
B = np.array([[1, 1], [-1, 1]])
K_inv = np.linalg.inv(K)


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def regul(v_d, th_d, th):  # compute the motor control signal
    # v_d: desired speed (m/s)
    # th_d: desired heading (rad)
    # th: current heading (rad)
    K = 1  # controller gain
    w = K * sawtooth(th_d - th)  # angular speed
    mat = np.linalg.inv(B) @ np.array([[v_d], [w]])
    ul, ur = mat.flatten()  # left right
    return ul, ur  # in m/s


def follow_point(b, m):  # compute the desired heading to follow the point b
    # b: target position
    # m: current position
    return np.arctan2(b[1, 0] - m[1, 0], b[0, 0] - m[0, 0])


def follow_line(a, b, m):  # compute the desired heading to follow the line [ab)
    # a: beginning of the line
    # b: end of the line
    # m: current position
    r = 8  # gain
    u = (b - a) / np.linalg.norm(b - a)  # unit vector of the line
    e = np.linalg.det(np.hstack([u, m - a]))  # distance error with the line
    phi = np.arctan2(b[1, 0] - a[1, 0], b[0, 0] - a[0, 0])  # orientation of the line
    ang = sawtooth(phi - np.arctan(e / r))  # desired heading
    # print("desired ANG (deg)", ang * 180 / np.pi)
    return ang


def lissajou_trajectory(c, R, T, t, i, N):
    # compute the Lissajou trajectory
    # c : center of the curve
    # R : amplitude of the curve
    # T : period of the trajectory
    # t : current time since the beginning of the mission
    # i : id of the robot
    # N : total number of robots
    phi1, phi2 = t * np.pi * 2 / T + 2 * i * np.pi / N, 2 * (t * np.pi * 2 / T + 2 * i * np.pi / N)
    pd = c + R * np.array([[np.cos(phi1)],
                           [np.sin(phi2)]])
    pd_dot = R * np.array([[-np.pi * 2 / T * np.sin(phi1)],
                           [np.pi * 4 / T * np.cos(2 * phi2)]])
    pd_ddot = R * np.array([[-(np.pi * 2 / T) ** 2 * np.cos(phi1)],
                            [-(np.pi * 4 / T) ** 2 * np.sin(phi2)]])
    return pd, pd_dot, pd_ddot


def control_lisssajou(p, th, pd, min_v=0.1, max_v=10):
    # compute the control signal (u1,u2) to follow a Lissajou curve with only distance error
    # p: current position (local coord)
    # th: current heading (rad)
    # pd: desired position (m) [2D vector]
    # min_v: min speed (m/s)
    # max_v: max speed (m/s)

    th_d = follow_point(pd, p)  # follow the line [mX)
    dist_error = np.linalg.norm(pd - p)
    v = min((1 + 0.1 * dist_error ** 2) * min_v, max_v)  # speed depends on quadratic error and is saturated
    return regul(v, th_d, th)


def control_feedback_linearization(pd, pd_dot, pd_ddot, dt, p, v, th, qx, qy):
    # compute the control signal (u1,u2) to follow the desired trajectory
    # (pd,pd_dot,pd_ddot) desired position, velocity, acceleration (in x , y frame)
    # (p,p_dot) : measured position and velocity
    # (qx,qy): estimated speed bias caused by the wind
    # dt: controller period

    if abs(v) < 0.1:  # avoid singularity
        # ~ print("speed singularity in controller")
        return np.array([[2, 0]]).T

    e = p - pd  # position error
    ed = np.array([[v * cos(th) + qx, v * sin(th) + qy]]).T - pd_dot  # velocity error
    A = np.array([[cos(th), -v * sin(th)], [sin(th), v * cos(th)]])  # transition matrix, p_ddot = A*u
    kc = 0.1
    u = np.linalg.inv(A) @ (pd_ddot - 2 * kc * ed - kc * e)  # [scalar acceleration, angular velocity]
    return u

def control_feedback_linearization2(pd, pd_dot, pd_ddot, dt, p, v, th, qx, qy,r=4):
    # as above but modified. the robot wait if ahead of the reference bellow r meters of distance

    if np.linalg.norm(pd-p)<r and np.dot(p-pd,pd_dot)>0:
        return np.zeros((2,1))
    else:
        return control_feedback_linearization(pd, pd_dot, pd_ddot, dt, p, v, th, qx, qy)


def control_station_keeping(a, d, pos, th, v=1):  # controller to keep the robot next ot a position
    # a: reference position (local position, m)
    # d: max distance desired (m)
    # pos : current position of the robot (local position, m)
    # th:  current heading (rad)
    # v: desired speed (m/s)

    d_mes = np.linalg.norm(pos - a)
    if d_mes > d:
        th_d = follow_point(a, pos)
        return regul(v, th_d, th)
    else:
        return 0, 0


###############################################
# LINK WITH THE MOTORS

def convert_motor_control_signal(u, v_hat, wmLeft, wmRight, cmdL_old, cmdR_old,
                                 dt):  # compute pwd control signal for the motors
    # u: controller output [v_dot,w] (m/s^2), (rad/s)
    # v_hat: boat estimated speed given by the propeller (m/s)
    # wmLeft, wmRight : measured rotation speed of the motors (turn/sec)
    k2, kpwm = 0.5, 2
    thust = u[0,0] + k2*v_hat # desired thurst, can only be positive
    if thust < 0: # can't go backward
        if u[1,0]>0:
            return(0,200)
        else:
            return(200,0)
    mat = K_inv @ np.array([[thust,u[1,0]]]).T
    wmLeft_d, wmRight_d = mat.flatten()
    # saturation between 0 and 200 turn / sec
    if wmLeft_d < 0 or wmRight_d < 0:  # lower motor saturation, the propellers can't go backward
        min_motor = min(wmLeft_d, wmRight_d)
        wmLeft_d, wmRight_d = wmLeft_d - min_motor+50, wmRight_d - min_motor+50
    if wmLeft_d > 200 or wmRight_d > 200:  # upper motor saturation
        max_motor = max(wmLeft_d, wmRight_d)
        wmLeft_d = 200*wmLeft_d/max_motor
        wmRight_d = 200*wmRight_d/max_motor

    # discrete proportional corrector for Pwm
    cmdL = min(max(cmdL_old + dt * kpwm * (wmLeft_d - wmLeft),0),200)
    cmdR = min(max(cmdR_old + dt * kpwm * (wmRight_d - wmRight),0),200)
    return cmdL, cmdR  # controlled PWM


#####################################""
# general control

class ControlBlock:
    def __init__(self, dt,traj0,r=4):
        self.state = 0  # 0 : move towards waypoint , 1 : move towards desired heading, 2 : stop
        self.dt = dt
        self.pd0 = np.reshape(np.array([traj0["pd"]]), (2, 1))
        self.pd_dot0 = np.reshape(np.array([traj0["pd_dot"]]), (2, 1))
        self.th_d0 = atan2(self.pd_dot0[1, 0], self.pd_dot0[0, 0])
        self.pd_ddot0 = np.reshape(np.array([traj0["pd_ddot"]]), (2, 1))
        self.r = r # radius of the station keeping circle

    def variable_update(self,p,v,th,qx,qy,wmLeft,wmRight,cmdL_old,cmdR_old):
        self.p = p
        self.v = v
        self.th = th
        self.qx = qx
        self.qy = qy
        self.wmLeft = wmLeft
        self.wmRight = wmRight
        self.cmdL_old = cmdL_old
        self.cmdR_old = cmdR_old
        return

    def follow_reference(self,pd, pd_dot, pd_ddot):  # make control_feedback_linearization and convert_motor_control_signal
        u = control_feedback_linearization(pd, pd_dot, pd_ddot, self.dt, self.p, self.v, self.th, self.qx, self.qy)
        cmdL, cmdR = convert_motor_control_signal(u, self.v, self.wmLeft, self.wmRight, self.cmdL_old, self.cmdR_old, self.dt)
        return cmdL, cmdR, u

    def follow_reference2(self,pd, pd_dot, pd_ddot,r=4): # same as above with control_feedback_linearization2
        u = control_feedback_linearization2(pd, pd_dot, pd_ddot, self.dt, self.p, self.v, self.th, self.qx, self.qy,r)
        cmdL, cmdR = convert_motor_control_signal(u, self.v, self.wmLeft, self.wmRight, self.cmdL_old, self.cmdR_old,
                                                  self.dt)
        return cmdL, cmdR, u

    def station_keeping1(self): # basic station keeping with in and out circle
        # change state
        if self.state == 0 and np.linalg.norm(self.pd0 - self.p) < self.r/2:
            state = 1
        if self.state == 1 and np.linalg.norm(self.pd0 - self.p) > self.r:
            state = 0

        # control
        if self.state == 0:
            return self.follow_reference(self.pd0, self.pd_dot0, self.pd_ddot0)
        if self.state == 1:
            return 0, 0, np.zeros((2, 1))

    def station_keeping2(self):
        # advanced station keeping to have correct heading

        # change state
        if self.state == 0:
            pd1 = self.pd0 - 0.5 * self.r * self.pd_dot0 / np.linalg.norm(self.pd_dot0)  # desired position for state 0
            if np.linalg.norm(pd1 - self.p) < self.r/2:
                self.state = 1
        if self.state == 1:
            e = sawtooth(self.th_d0 - self.th)
            if abs(e) < 0.1:
                self.state = 2
        if self.state == 2 and np.linalg.norm(self.pd0 - self.p) > self.r:
            self.state = 0

        # compute control
        if self.state == 0: # go towards pd1
            return self.follow_reference(pd1, self.pd_dot0, self.pd_ddot0)
        if self.state == 1: # turn towards th_d heading
            u = np.array([[0, 0.1 * e]]).T
            return convert_motor_control_signal(u, self.v, self.wmLeft, self.wmRight, self.cmdL_old, self.cmdR_old, self.dt), u
        if self.state == 2: # don't move
            return 0, 0, np.zeros((2, 1))
