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
from geometry_msgs.msg import PoseStamped
from spc_ads_pub.msg import CableLength
from scampi_ks_ros.srv import *
#Import the Ceres SCAMPI kinematic solver
import scampi_ks_solver
import pyquaternion

#Ros Service to initlize cable lengths given the equilibrium poses and spc lengths
def handle_init_cable_lengths(req):
    solver_class.initLengthUpdate(req.eq_pose, np.array(req.eq_spc_len))
    print('Cable lengths initialized to:')
    print(solver_class.init_length)
    return 1

def handle_set_pose_source(req):
    source = {0:'solver_output', 1:'prop_topic'}
    print(f'Pose Source is set to: {source[req.source_id]}')
    
    if req.source_id == 0:
        solver_class.standalone_mode = True
    elif req.source_id == 1:
        solver_class.standalone_mode = False
    else:
        solver_class.standalone_mode = True

    return 1

# Get the pose estimates from the MARS EKF
def pose_callback(msg):
    solver_class.poseUpdate(msg)

# Get cable length measurements
def cable_length_callback(msg):
    solver_class.measUpdate(msg)
    
# Start the node
rospy.init_node('scampi_ks_node')
print("Node Initialized")
#Load the ros params
param_mng = ParamManager()
param_mng.loadParams()

#The publisher for outputing the FK result
pose_publisher = rospy.Publisher('/fk_pose', PoseStamped, queue_size=1)

# Load the solver parameters
robot_params, state_params = loadRobotParams(param_mng.params['~robot_config'])
print('Robot parameters are loaded!')

# Instantiate the solver class
dT = 1/param_mng.params['~rate']
#default_mode = (True if param_mng.params['~mode'] == 'standalone' else False)

solver_class = ScampiSolver(robot_params, state_params, standalone_mode = True,
                            sampling_time = dT, result_publisher = pose_publisher)
      

#Subscribe to the cable length topic
rospy.Subscriber("/cable_len", CableLength, cable_length_callback)

#Subscribe to the MARS pose Estimates
rospy.Subscriber("/ekf_pose_prop", PoseStamped, pose_callback)

s = rospy.Service('/scampi_ks_ros/init_cable_lengths', CableLenInit, handle_init_cable_lengths)
s2 = rospy.Service('/scampi_ks_ros/set_pose_source', SetPoseSource, handle_set_pose_source)
print('ros service added')
initLenFlag = True
try:
#     while not rospy.is_shutdown():
#         if initLenFlag and (solver_class.latest_eq_len is not None) and (solver_class.latest_eq_pose is not None):
#             solver_class.initLengthUpdate(solver_class.latest_eq_pose, solver_class.latest_eq_len)
#             print(solver_class.latest_eq_pose, solver_class.latest_eq_len)
#             initLenFlag = False
#             print('Cable Length initilized!')

#         rospy.sleep(0.1)
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
