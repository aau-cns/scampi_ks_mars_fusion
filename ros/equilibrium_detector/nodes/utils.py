from __future__ import print_function
import roslib
import sys
import rospy
import os
import subprocess
import threading
import time
import yaml
import numpy as np
import pdb
#Import the messages
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from spc_ads_pub.msg import CableLength
#Import the Ceres SCAMPI kinematic solver
import scampi_ks_solver
import pyquaternion
from liegroups.numpy import SO3
import pickle

#ROS parameter manager
class ParamManager():
    def __init__(self):
        self.params=\
        {
            '~detector_params':'.'
        }
    def loadParams(self):
        for key in self.params.keys():
            if rospy.has_param(key):
                self.params[key]=rospy.get_param(key)
        return self.params

# A function that loads the robot parameters from the yaml file
def loadDetectorParams(config_file):
    '''A function that loads the robot parameters from the yaml file'''
    with open(config_file,'r') as f:
        configs = yaml.safe_load(f)

    return configs

# Solver Manager Class
class EqDetector():

    def __init__(self, detector_params, eq_publisher = None):
        self.params = detector_params
        self.eq_publisher = eq_publisher
        self.acc_window = []
        self.gyr_window = []
        self.enc_window = []
        self.imu_eq = False
        self.enc_eq = False
        self.eq_covariance = self.params['eq_covariance']['data']
        self.non_eq_covariance = self.params['non_eq_covariance']['data']
        self.dataset = []

    def poseUpdate(self, fk_pose_msg):
        print("New FK Pose Arrivied!")
        if fk_pose_msg.pose.orientation.w < 0:
            pose_msg.pose.orientation.w = -pose_msg.pose.orientation.w
            pose_msg.pose.orientation.x = -pose_msg.pose.orientation.x
            pose_msg.pose.orientation.y = -pose_msg.pose.orientation.y
            pose_msg.pose.orientation.z = -pose_msg.pose.orientation.z

        msg = PoseWithCovarianceStamped()
        msg.header = fk_pose_msg.header
        #Positive Shift in the time stamp
        msg.header.stamp = rospy.Time.from_sec(fk_pose_msg.header.stamp.to_sec()-0.0)
        msg.pose.pose = fk_pose_msg.pose

        if self.enc_eq:
            print('Sending eq covariance')
            msg.pose.covariance = self.eq_covariance
        else:
            print('sending non eq covariance')
            msg.pose.covariance = self.non_eq_covariance

        self.eq_publisher.publish(msg)


    def cableLenUpdate(self, msg):
        cable_length = np.array(msg.length).reshape(4,1)
        len_mag = np.linalg.norm(cable_length)

        self.enc_window.append(len_mag)
        if len(self.enc_window) > self.params['enc_window_len']:
            del self.enc_window[0]

        enc_var = np.var(self.enc_window)
        acc_var = np.var(self.acc_window)
        gyr_var = np.var(self.gyr_window)
        self.dataset.append([enc_var, gyr_var, acc_var])

        if len(self.dataset)>12500:
            with open('/home/errr0/data.pckl','wb') as f:
                print('saving the dataset file')
                pickle.dump(self.dataset,f)
                self.dataset=[]

#         print(f'cable len variance: {enc_var}')
        if enc_var < self.params['enc_thr']:
            self.enc_eq = True
        else:
            self.enc_eq = False

    def imuUpdate(self, msg):
        acc = np.array([msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        gyr = np.array([msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z])

        acc_mag = np.linalg.norm(acc)
        gyr_mag = np.linalg.norm(gyr)

        self.acc_window.append(acc_mag)
        if len(self.acc_window) > self.params['imu_window_len']:
            del self.acc_window[0]

        self.gyr_window.append(gyr_mag)
        if len(self.gyr_window) > self.params['imu_window_len']:
            del self.gyr_window[0]

        acc_var = np.var(self.acc_window)
        gyr_var = np.var(self.gyr_window)
#         print(f'gyro,accel vars: {gyr_var,acc_var}')

        if acc_var < self.params['acc_thr'] and gyr_var < self.params['gyr_thr']:
            self.imu_eq = True
        else:
            self.imu_eq = False
