import cv2
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import json
import os
import pickle
import rosbag
import os
import pandas as pd
from spc_ads_pub.msg import  CableLength
from sensor_msgs.msg import Imu
import argparse

parser = argparse.ArgumentParser('bag_to_csv', description='A simple script to extract the data recorded in the given bag file into several csv files')
parser.add_argument('bag_path', type = str, help='path to the bag file')
parser.add_argument('save_path', type = str, help='where to save the csv files')
args = parser.parse_args()

class rosBagDataExtractor():
    '''
    A class to abstract away the data extraction process from a bag file.
        Attributes
        ----------
        bag_file : str
            The path to the bag file
        save_path : str
            The directory/path where you want to save the extracted data
        topic_name : str
            The name of the ROS topic that you want extract.
        extract_row_fn: function(msg)
            A custom function that takes a message and returns a list of data extracted from it
        save_fn: function(data, path)
            A function that takes the list of extracted data and saves them in the given path.

        Methods
        -------
        run():
            Starts the extraction process.
    '''
    def __init__(self, bag_file, save_path, topic_name, extract_row_fn = None, save_fn = None):
        '''
        Constructs all the necessary attributes for the rosBagDataExtractor object.

        Parameters
        ----------
            bag_file : str
                The path to the bag file
            save_path : str
                The directory/path where you want to save the extracted data
            topic_name : str
                The name of the ROS topic that you want extract.
            extract_row_fn: function(msg)
                A custom function that takes a message and returns a list of data extracted from it
            save_fn: function(data, path)
                A function that takes the list of extracted data and saves them in the given path.
        '''
        self.bag_file = bag_file
        self.topic_name = topic_name
        self.extract_row_fn = extract_row_fn
        self.save_fn = save_fn
        self.save_path = save_path
        assert extract_row_fn!=None, 'Please enter a funstion to extract data from each message'
        try:
            self.bag = rosbag.Bag(self.bag_file)
        except:
            print('Error in opening the Bag file!')
            
    def run(self):
        '''
            Runs the data extraction process and saves the results.
        '''
        data = []
        for topic, msg, t in self.bag.read_messages(topics=[self.topic_name]):
            data.append(self.extract_row_fn(msg))
        if self.save_fn is not None:
            df = self.save_fn(data, self.save_path)
            return df
           
            
def extract_pose_row(msg):
    return  [msg.header.stamp.to_sec(), 
             msg.pose.position.x,
             msg.pose.position.y,
             msg.pose.position.z,
             msg.pose.orientation.w,
             msg.pose.orientation.x,
             msg.pose.orientation.y,
             msg.pose.orientation.z]

def save_pose_results(data, save_path):
    cols = ['stamp',
            'px',
            'py',
            'pz',
            'qw',
            'qx',
            'qy',
            'qz']
    df = pd.DataFrame(data, columns = cols)
    df.to_csv(save_path, index = False)
    return df


def extract_imu_row(msg):
    return  [msg.header.stamp.to_sec(), 
             msg.angular_velocity.x,
             msg.angular_velocity.y,
             msg.angular_velocity.z,
             msg.linear_acceleration.x,
             msg.linear_acceleration.y,
             msg.linear_acceleration.z]
def save_imu_results(data, save_path):
    cols = ['stamp',
            'gx',
            'gy',
            'gz',
            'ax',
            'ay',
            'az']
    df = pd.DataFrame(data, columns = cols)
    df.to_csv(save_path, index = False)
    return df


def extract_cable_len_row(msg):
    return  [msg.header.stamp.to_sec()] + list(msg.length)


def save_cable_len_results(data, save_path):
    cols = ['stamp',
            'l1',
            'l2',
            'l3',
            'l4']
    df = pd.DataFrame(data, columns = cols)
    df.to_csv(save_path, index = False)
    return df


def extract_bias_row(msg):
    return  [msg.header.stamp.to_sec()] + list([msg.b_a.x, msg.b_a.y, msg.b_a.z]) + list([msg.b_w.x, msg.b_w.y, msg.b_w.z])

def save_bias_results(data, save_path):
    cols = ['stamp',
            'bax',
            'bay',
            'baz',
            'bgx',
            'bgy',
            'bgz']
    df = pd.DataFrame(data, columns = cols)
    df.to_csv(save_path, index = False)
    return df

# What topics to listen to and what to extract from them
output_files = [
    'raw_imu.csv',
    'cable_len.csv',
    'gt_pose.csv',
    'sover_pose.csv',
    'spc_pose.csv',
    'mars_imu_pose.csv',
    'mars_center_pose.csv',
    'mars_calstate_pose.csv',
    'mars_bias_state.csv'
]

topics = [
          ['/mti/sensor/imu', 'imu'],
          ['/spc_ADS_node/spc_cable_length', 'cable_len'], 
          ['/dh_optitrack/CustomDolly/pose', 'pose'], 
          ['/scampi_ks_ros/fk_pose', 'pose'], 
          ['/spc_ADS_node/spc_dolly_pose', 'pose'], 
#          ['/mars_pose_covariance_node/pose_state_out', 'pose'], 
          ['/mars_pose_node/pose_state_out', 'pose'], 
          ['/mars_pose_covariance_node/pose_state_out_center', 'pose'],
          ['/mars_pose_covariance_node/pose_cal_state_out', 'pose'],
          ['/mars_pose_covariance_node/full_state_out', 'cal_state']
         ]

extractor_fn = {'pose':extract_pose_row,
                'cable_len':extract_cable_len_row,
                'cal_state':extract_bias_row,
                'imu':extract_imu_row}

save_fn = {'pose':save_pose_results,
           'cable_len':save_cable_len_results,
           'cal_state':save_bias_results,
           'imu':save_imu_results}

for topic_idx in range(len(topics)):
    print(topics[topic_idx][0])

    bext = rosBagDataExtractor(args.bag_path, os.path.join(args.save_path,output_files[topic_idx]), 
                               topics[topic_idx][0], 
                               extractor_fn[topics[topic_idx][1]], 
                               save_fn[topics[topic_idx][1]])
    df_pose = bext.run()
