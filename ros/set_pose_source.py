from scampi_ks_ros.srv import *
from geometry_msgs.msg import PoseStamped
import rospy
import argparse

parser = argparse.ArgumentParser(description = 'A simple script to set the chosen pose source of the scampi_ros_ks solver.')
parser.add_argument('source', type=int, help='The selected pose source is: 0 -> standalnone, 1 -> EKF')
args = parser.parse_args()
set_pose_source = rospy.ServiceProxy('/scampi_ks_ros/set_pose_source', SetPoseSource)

set_pose_source(args.source)
