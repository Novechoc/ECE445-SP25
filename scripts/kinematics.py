#!/usr/bin/env python

'''

lab5pkg_ik/lab5_func.py

@brief: functions for computing forward and inverse kinematics of UR3e robot arm
@author: Songjie Xiao
@date: Monday 2023/3/20

'''

import numpy as np
import math
from scipy.linalg import expm

PI = np.pi

OFFSET = 0.25

OFFSET_CAM = 0.12
OFFSET_CAM_H = 0.0356

def abs(a):
    return a if a > 0 else -a

def sign(a):
    if a == 0:
        return 1
    return a/abs(a)

def sign_off_mod(a, mod):
    assert mod > 0, "The mod has too be greater than 0"
    return a % mod if a >= 0 else a % mod - mod

def soff_mod_and_rad(a, mod):
    return sign_off_mod(a, mod) / 180 * PI

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
Add any helper functions as you need.
"""
def Get_MS():
    S1 = np.array([0, 0, 1, 
                    0, 0, 0])
    S2 = np.array([1, 0, 0, 
                    0, 0.15185, 0]) 
    S3 = np.array([1, 0, 0, 
                    0, 0.15185, 0.24355])
    S4 = np.array([1, 0, 0, 
                    0, 0.15185, 0.24355+0.2132])
    S5 = np.array([0, -1, 0, 
                    0.15185, 0, -0.13105])
    S6 = np.array([1, 0, 0, 
                    0, 0.15185, 0.24355+0.2132+0.08535])

    # Stack screw axes into matrix S
    S = np.stack((S1, S2, S3, S4, S5, S6), axis=1)

    # Define M matrix - the home configuration
    M = np.array([[0, 0, 1, 0.120-0.093+0.104+0.092+OFFSET],
                [0, 1, 0, -0.5421],
                [-1, 0, 0, 0.15185],
                [0, 0, 0, 1]])
    return M, S

def mat_S(S):
    return np.array([
                    [0, -S[2], S[1], S[3]],
                    [S[2], 0, -S[0], S[4]],
                    [-S[1], S[0], 0, S[5]],
                    [0, 0, 0, 0]
                    ])

def matrix_exp(S, theta):
    return expm(np.dot(mat_S(S), theta))
"""
Function that calculates encoder numbers for each motor
"""
def forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6):
    theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
    T = np.eye(4)

    M, S = Get_MS()

    # Convert true input thetas to our own thetas
    theta = [theta1-PI/2, theta2, theta3, theta4+PI/2, theta5, theta6]

    # Initialize the transformation matrix
    T = np.eye(4)

    # Compute the transformation matrix
    for i in range(6):
        T = np.dot(T, matrix_exp(S[:, i], theta[i]))
    T = np.dot(T, M)
    
    # print(str(T) + "\n")
    return [float(i) for i in T[:-1, -1]]

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def top_inverse_kinematics(xWgrip, yWgrip, zWgrip, yaw_WgripDegree=0.0):
    # theta1 to theta6
    thetas = [0.0, 0.0, 0.0, 0.0, 0.0]

    l01 = 0.15185
    l02 = 0.120
    l03 = 0.24355
    l04 = 0.093
    l05 = 0.2132
    l06 = 0.104
    l07 = 0.08535
    l08 = 0.0921
    l09 = OFFSET # TODO: lenth of the gripper

    # Convert yaw angle to radians
    yaw = yaw_WgripDegree * PI / 180.0

    # Convert world coordinates to robot coordinates and get wrist center from grip position
    xcen = -yWgrip
    ycen = xWgrip
    zcen = zWgrip

    # Calculate theta1 (base rotation)
    lp = l02 - l04 + l06
    thetas[0] = np.arctan2(ycen, xcen) - np.arcsin(lp / np.sqrt(xcen**2 + ycen**2))
    
    # Calculate x3end, y3end, z3end (position of joint 3)
    x3end = xcen - l07 * np.cos(thetas[0]) + lp * np.sin(thetas[0])
    y3end = ycen - l07 * np.sin(thetas[0]) - lp * np.cos(thetas[0])
    z3end = zcen + l08 + l09
    
    # Calculate theta2 and theta3
    d = np.sqrt(x3end**2 + y3end**2 + (z3end - l01)**2)
    thetas[1] = - (np.arccos((l03**2 + d**2 - l05**2) / (2*l03*d)) + np.arctan2(z3end - l01, np.sqrt(x3end**2 + y3end**2)))
    thetas[2] = np.pi - np.arccos((l03**2 + l05**2 - d**2) / (2*l03*l05))

    # For theta4, we need it to keep the end effector vertical
    thetas[3] = -(thetas[1] + thetas[2]) - PI/2

    # theta5 keeps the end effector vertical
    thetas[4] = -PI/2

    # Normalize angles to be within [-PI, PI]
    for i in range(len(thetas)):
        while thetas[i] > PI:
            thetas[i] -= 2*PI
        while thetas[i] < -PI:
            thetas[i] += 2*PI
        
    thetas[0] = thetas[0] + PI/2

    # print("Inverse kinematics calculated: " + str(thetas) + "\n")
    ret = []
    for t in thetas:
        if np.isnan(t):
            return None
        ret.append(float(t))
    return ret

def side_inverse_kinematics(xWgrip, yWgrip, zWgrip, yaw_WgripDegree=0.0):
    # theta1 to theta6
    thetas = [0.0, 0.0, 0.0, 0.0, 0.0]

    l01 = 0.15185
    l02 = 0.120
    l03 = 0.24355
    l04 = 0.093
    l05 = 0.213
    l06 = 0.104
    l07 = 0.085
    l08 = 0.092
    l09 = OFFSET # TODO: lenth of the gripper

    # Convert yaw angle to radians
    yaw = soff_mod_and_rad(yaw_WgripDegree, 180)
    print('yaw in rad:', yaw)

    # Convert world coordinates to robot coordinates and Calculate wrist center from grip position
    xcen = -yWgrip - (l08+l09) * np.cos(yaw)
    ycen = xWgrip - (l08+l09) * np.sin(yaw)
    zcen = zWgrip

    # Calculate theta1 (base rotation)
    lp = l02 - l04 + l06
    thetas[0] = np.arctan2(ycen, xcen) - np.arcsin(lp / np.sqrt(xcen**2 + ycen**2))
    
    # Calculate x3end, y3end, z3end (position of joint 3)
    x3end = xcen + lp * np.sin(thetas[0])
    y3end = ycen - lp * np.cos(thetas[0])
    z3end = zcen + l07
    
    # Calculate theta2 and theta3
    d = np.sqrt(x3end**2 + y3end**2 + (z3end - l01)**2)
    thetas[1] = - (np.arccos((l03**2 + d**2 - l05**2) / (2*l03*d)) + np.arctan2(z3end - l01, np.sqrt(x3end**2 + y3end**2)))
    thetas[2] = np.pi - np.arccos((l03**2 + l05**2 - d**2) / (2*l03*l05))

    # For theta4, we need it to keep the end effector horizontal
    thetas[3] = -(thetas[1] + thetas[2])

    # theta5 adjusts the yaw angle
    thetas[4] = -yaw + thetas[0] + PI/2

    thetas[0] += PI/2

    # print("Inverse kinematics calculated: " + str(thetas) + "\n")
    ret = []
    for t in thetas:
        # Normalize angles to be within [-PI, PI]
        while t > PI:
            t -= 2*PI
        while t < -PI:
            t += 2*PI
        if np.isnan(t):
            return None
        ret.append(float(t))
    return ret
