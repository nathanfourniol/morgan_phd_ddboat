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
from math import sin, cos

k1, k3 = 0.01,0.003#150, 50
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
        print("speed singularity in controller")
        return np.array([[2, 0]]).T

    e = p - pd  # position error
    ed = np.array([[v * cos(th) + qx, v * sin(th) + qy]]).T - pd_dot  # velocity error
    A = np.array([[cos(th), -v * sin(th)], [sin(th), v * cos(th)]])  # transition matrix, p_ddot = A*u
    kc = 0.1
    u = np.linalg.inv(A) @ (pd_ddot - 2 * kc * ed - kc * e)  # [scalar acceleration, angular velocity]

    # # saturation of the acceleration
    # if abs(u[0,0])>2:
    #     print("saturation of the acceleration ",u[0,0])
    #     u[0,0] =np.sign(u[0,0])*2
    #
    # # saturation of the angular velocity
    # if abs(u[1,0])>5:
    #     print("saturation of the angular velocity")
    #     u[1,0] =np.sign(u[1,0])*5
    return u


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
    k2, kpwm = 0.5, 2#7.5  # TODO verify k2
    # ~ print(" ")
    # ~ print("u is ",u.T)
    # ~ print("k2*v_hat is",k2*v_hat)
    thust = u[0,0] + k2*v_hat # desired thurst, can only be positive
    if thust < 0: # can't go backward
        print("can't go backward")
        # ~ thust = -0*thust
        # ~ u[1,0] = -10*u[1,0]
        if u[1,0]>0:
            return(200,0)
        else:
            return(0,200)
    # ~ print("vd wd is",np.array([[thust,u[1,0]]]))
    mat = K_inv @ np.array([[thust,u[1,0]]]).T
    # ~ mat = K_inv @ (u + np.array([[k2 * v_hat, 0]]).T)  # convert to propeller rotation speed (turn/sec
    wmLeft_d, wmRight_d = mat.flatten()
    # ~ print("wm is ",wmLeft_d,wmRight_d)
    # saturation between 0 and 200 turn / sec
    # ~ print(u.T)
    if wmLeft_d < 0 or wmRight_d < 0:  # lower motor saturation, the propellers can't go backward
        # ~ print("lower motor saturation")
        min_motor = min(wmLeft_d, wmRight_d)
        wmLeft_d, wmRight_d = wmLeft_d - min_motor+50, wmRight_d - min_motor+50
        # ~ print(wmLeft_d,wmRight_d)
    if wmLeft_d > 200 or wmRight_d > 200:  # upper motor saturation
        # ~ print("upper motor saturation")
        max_motor = max(wmLeft_d, wmRight_d)
        #wmLeft_d, wmRight_d = 200 + wmLeft_d - max_motor, 200 + wmRight_d - max_motor
        wmLeft_d = 200*wmLeft_d/max_motor
        wmRight_d = 200*wmRight_d/max_motor
        # ~ print(wmLeft_d,wmRight_d)
    # ~ wmLeft_d, wmRight_d = 50,70
    # ~ print("after saturation, wm is ", wmLeft_d,wmRight_d)
    # ~ print("measured wm is",wmLeft,wmRight)
    # ~ print("total control error on propellers is",abs(wmLeft_d-wmLeft)+abs(wmRight_d-wmRight))

    # discrete proportional corrector for Pwm
    cmdL = min(max(cmdL_old + dt * kpwm * (wmLeft_d - wmLeft),0),200)
    cmdR = min(max(cmdR_old + dt * kpwm * (wmRight_d - wmRight),0),200)
    # ~ print("pwm is ", cmdL,cmdR)
    return cmdL, cmdR  # controlled PWM
