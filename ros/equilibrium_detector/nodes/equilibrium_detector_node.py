#!/usr/bin/env python3
from __future__ import print_function
#Import the standard libs
import roslib
import sys
import rospy
import os
import subprocess
import threading
import time
import yaml
import numpy as np
#Import the helper funcitons
from utils import *
#Import the messages
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from spc_ads_pub.msg import CableLength

# Get the latest FK solution from the solver
def pose_callback(msg):
    detector_class.poseUpdate(msg)

# Get cable length measurements
def cable_length_callback(msg):
    detector_class.cableLenUpdate(msg)

# Get cable length measurements
def imu_callback(msg):
    detector_class.imuUpdate(msg)
    
# Start the node
rospy.init_node('equilibrium_detector_node')
print("Node Initialized")
#Load the ros params
param_mng = ParamManager()
param_mng.loadParams()

#The publisher for outputing the FK result
eq_publisher = rospy.Publisher('/eq_pose', PoseWithCovarianceStamped, queue_size=1)

# Load the detector parameters
detector_params = loadDetectorParams(param_mng.params['~detector_params'])
print('Detector parameters are loaded!')

detector_class = EqDetector(detector_params, eq_publisher = eq_publisher)
      

#Subscribe to the cable length topic
rospy.Subscriber("/cable_len", CableLength, cable_length_callback)

#Subscribe to the Solver pose Estimates
rospy.Subscriber("/fk_pose", PoseStamped, pose_callback)

#Subscribe to the IMU Raw Values
rospy.Subscriber("/imu", Imu, imu_callback)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
