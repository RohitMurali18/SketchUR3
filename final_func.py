#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from final_header import *
import math
from modern_robotics import FKinSpace

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

# =================== Your code starts here ====================#
 # Fill in the correct values for S1~6, as well as the M matrix
def Get_MS():
    M = np.array([[0, -1, 0, 390], [0, 0, -1, 401], [1, 0, 0, 215.5], [0, 0, 0, 1]])
    S = np.zeros((6, 6))
    S[:,0] = [0, 0, 1, 150, 150, 0]
    S[:,1] = [0, 1, 0, -162, 0, -150]
    S[:,2] = [0, 1, 0, -162, 0, 94]
    S[:,3] = [0, 1, 0, -162, 0, 307] 
    S[:,4] = [1, 0, 0, 0, 162, -260]
    S[:,5] = [0, 1, 0, -162, 0, 390]
 # ==============================================================#
    return M, S

L1=0.152
L2=0.12
L3=0.244
L4=0.093
L5=0.213
L6=0.083
L7=0.083
L8=0.082
L9=0.0535

def w2bframe(xWgrip, yWgrip,zWgrip):
    # =================== Your code starts here ====================#
	xBframe, yBframe,zBframe = xWgrip+0.150,yWgrip-0.150,zWgrip -0.010
	# ==============================================================#
	return xBframe, yBframe, zBframe

def theta1fun(xcen,ycen):
    thetaA=math.atan2(ycen,xcen)
    A= math.sqrt(xcen**2 + ycen**2)
    B = 0.120-0.093+0.083
    theta1=thetaA-math.asin(B/A)
    print(theta1)
    return theta1

def theta6fun(theta1,yaw):
    theta6=90*math.pi/180-yaw+theta1
    return theta6

def wrist2center(xBframe, yBframe,zBframe,yaw):
    x_cen= xBframe-0.0535*math.cos(yaw)
    y_cen= yBframe-0.0535*math.sin(yaw)
    z_cen = zBframe

    
    return x_cen,y_cen,z_cen


def cent23end(x_cen,y_cen,z_cen,theta1):
    y_3end = y_cen - (0.027+0.083)*(np.cos(theta1)) - 0.083 * np.sin(theta1)
    x_3end = x_cen - 0.083 * np.cos(theta1) + (0.083+0.027)*np.sin(theta1)
    z_3end = z_cen + 0.059 + 0.082
    return x_3end,y_3end,z_3end

def calculate_triangle_angle(a: float, b: float, c: float) -> float:
    cos_angle = (a * a + b * b - c * c) / (2.0 * a * b)
    cos_angle=max(min(cos_angle,1.0),-1.0)
    return math.acos(cos_angle)


def theta2fun(x_3end: float, y_3end: float, z_3end: float) -> float:
    proj_len = math.sqrt(x_3end * x_3end + y_3end * y_3end)
    # Calculate vertical offset
    z_offset = L1 - z_3end
    # Calculate distance to end point
    c = math.sqrt(proj_len * proj_len + z_offset * z_offset)
    # Calculate angle using law of cosines
    print(L3, c, L5)
    print('22222222222222222222222222222222222222222222')
    angle_A = calculate_triangle_angle(L3, c, L5)
    # print(L3, c, L5)
    # Calculate angle from horizontal
    angle_B = math.atan2(z_offset, proj_len)
    # Combine angles for final θ₂
    return angle_A + angle_B

def theta3fun(x_3end: float, y_3end: float, z_3end: float) -> float:
    # Calculate projection length in x-y plane
    proj_len = math.sqrt(x_3end * x_3end + y_3end * y_3end)
    # Calculate vertical offset
    z_offset = L1 - z_3end
    # Calculate distance to end point
    a = math.sqrt(proj_len * proj_len + z_offset * z_offset)
    # Calculate interior angle using law of cosines
    interior_angle = calculate_triangle_angle(L5, L3, a)
    # θ₃ is the supplement of the interior angle
    return math.pi - interior_angle


def theta4fun(theta3,theta2):
    theta4=-theta3-theta2
    # theta4 = math.radians(theta4)
    return -theta4

"""
Function that calculates encoder numbers for each motor
"""

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    M, S = Get_MS()
    return_value = [None] * 6
    thetaList = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
    T = FKinSpace(M, S, thetaList.flatten())  # This already computes full transformation
    print(str(T) + "\n")
    
    # Apply joint angle offsets
    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5 * PI)
    return_value[4] = theta5
    return_value[5] = theta6
    
    return return_value

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    M,S=Get_MS()
    # =================== Your code starts here ====================#
    
    xBframe, yBframe,zBframe= w2bframe(xWgrip, yWgrip,zWgrip)
    yaw_WgripDegree = yaw_WgripDegree*math.pi/180
    x_cen,y_cen,z_cen=wrist2center(xBframe, yBframe,zBframe,yaw_WgripDegree)
    theta1 = theta1fun(x_cen, y_cen)
    theta6= theta6fun(theta1,yaw_WgripDegree)
    x_3end,y_3end,z_3end=cent23end(x_cen,y_cen,z_cen,theta1)
    theta2= -theta2fun(x_3end,y_3end,z_3end) 
    theta3= theta3fun(x_3end,y_3end,z_3end)
    theta4= -theta4fun(theta3,theta2)
    theta5=-math.radians(90)
    
    return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
