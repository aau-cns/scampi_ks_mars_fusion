from spc_ads_pub.msg import CableLength
from scampi_ks_ros.srv import CableLenInit
from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy

init_cable_lengths = rospy.ServiceProxy('/scampi_ks_ros/init_cable_lengths', CableLenInit)

'''
eq_pose = PoseStamped()
eq_pose.pose.orientation.x = 0.0202
eq_pose.pose.orientation.y = 0.0631
eq_pose.pose.orientation.z = -0.0582
eq_pose.pose.orientation.w = 0.9961
eq_pose.pose.position.x = 0.0402
eq_pose.pose.position.y = -1.1387
eq_pose.pose.position.z = 3.9743

eq_len = [8.6661,    8.8732,    7.8163,    7.5852]
'''

'''
Eq position of 2D_CD_TC21_1_2020-12-01-16-56-00.bag
  position:
    x: 0.309187084436
    y: -1.83724963665
    z: 2.18375515938
  orientation:
    x: 0.00333187263459
    y: 0.0209373664111
    z: 0.0566803403199
    w: -0.998167276382

length: [9.310307588623015, 9.339680311771485, 9.351678330640288, 9.32696962228383]

'''
eq_pose = PoseStamped()
eq_pose.pose.orientation.x = -0.00333187263459
eq_pose.pose.orientation.y = -0.0209373664111
eq_pose.pose.orientation.z = -0.0566803403199
eq_pose.pose.orientation.w = 0.998167276382
eq_pose.pose.position.x = 0.309187084436
eq_pose.pose.position.y = -1.83724963665
eq_pose.pose.position.z = 2.18375515938

eq_len = [9.310307588623015, 9.339680311771485, 9.351678330640288, 9.32696962228383]

init_cable_lengths(eq_pose, eq_len)
