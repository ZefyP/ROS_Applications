#! /usr/bin/env python3
# search_server.py

# Import the core Python modules for ROS and to implement ROS Actions:
from soupsieve import closest
import rospy
import actionlib


# Import some other useful Python Modules
from math import sqrt, pow


x0= 4.848623
y0=  0.565769
z0= -0.000105

x=6.414e-07
y= 1.184e-06
z= 0.0


# determine how far the robot has travelled so far:
distance = sqrt(pow(x0 -x, 2) + pow(y0 -y, 2))
print(f"    ***** The answer is = {distance} meters")
