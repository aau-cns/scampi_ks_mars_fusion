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
from geometry_msgs.msg import PoseStamped
from spc_ads_pub.msg import CableLength
#Import the Ceres SCAMPI kinematic solver
import scampi_ks_solver
import pyquaternion
from liegroups.numpy import SO3


#ROS parameter manager
class ParamManager():
    def __init__(self):
        self.params=\
        {
            '~rate':6,
            '~robot_config':'.'
        }
    def loadParams(self):
        for key in self.params.keys():
            if rospy.has_param(key):
                self.params[key]=rospy.get_param(key)
        return self.params

# A function that loads the robot parameters from the yaml file
def loadRobotParams(config_file):
    '''A function that loads the robot parameters from the yaml file'''
    with open(config_file,'r') as f:
        configs = yaml.safe_load(f)

    pulleys     = np.array(configs['pulleys']['data']).reshape(4, 3)
    anchors     = np.array(configs['anchors']['data']).reshape(4, 3)
    r_to_cog    = np.array(configs['r_to_cog']['data']).reshape(3, 1)
    fg          = configs['fg']
    gc          = configs['gc']

    robot_params = scampi_ks_solver.CableRobotParams(gc, fg)
    robot_params.setCog(r_to_cog)

    robot_params.setEEAnchors(anchors[0,:],anchors[1,:],anchors[2,:],anchors[3,:])
    robot_params.setPulleyPoses(pulleys[0,:],pulleys[1,:],pulleys[2,:],pulleys[3,:])

    state_params = None
    return robot_params, state_params

# Solver Manager Class
class ScampiSolver():

    def __init__(self, robot_params, state_params, standalone_mode = True, sampling_time = 0.1, result_publisher = None):
        self.robot_params = robot_params
        self.result_publisher = result_publisher
        self.sampling_time = sampling_time

        self.robot_params = robot_params
#         self.init_length = state_params['init_length']

        self.best_known_pose = None
        self.latest_cable_length_stamp = None
        self.latest_pose_from_mars = None
        self.latest_eq_pose = None
        self.latest_eq_len = None
        self.last_exec_stamp = rospy.Time.now().to_sec()
        self.latest_cable_length = np.zeros((4,1))

        self.solver_ready = False
        self.pose_recent_enough = False
        self.init_length_ready = False
        # In standalone mode, we disregard the porpagated poses from the MARS and use the solver node as a
        # standalone forwrad-kinematics pose measurement.
        self.standalone_mode = standalone_mode
        self.cable_length_window = []
        self.cable_length_var = 1e8
        self.init_length_ready = False


    def initLengthUpdate(self, eq_pose, eq_len):
        '''
        Given the the equilibrium cable lengths and poses, compute the correcting
        Cable length value.
        '''
        eq_rot,eq_pos = self.poseMsgToRt(eq_pose)
        #Using the first solver, get the expected cable length values
        R, lc_0, self.cable_forces, C1, C2, b_in_w \
            = scampi_ks_solver.inverseKinematicsSolver(self.robot_params, eq_pos, eq_rot)

        self.init_length = lc_0.reshape(4,1) - eq_len.reshape(4,1)
        # At the time of initilization, the best known pose is the eq_pose
        self.best_known_pose = eq_pose

        #The cable length is initlized now, therefore, we can allow the solvers to run in this regard
        self.init_length_ready = True
        return True

    def poseUpdate(self, pose_msg):
        #pdb.set_trace()
        #Flip the orientation based on what is done in the matlab implementation
        if pose_msg.pose.orientation.w < 0:
            pose_msg.pose.orientation.w = -pose_msg.pose.orientation.w
            pose_msg.pose.orientation.x = -pose_msg.pose.orientation.x
            pose_msg.pose.orientation.y = -pose_msg.pose.orientation.y
            pose_msg.pose.orientation.z = -pose_msg.pose.orientation.z

        # Transform IMU pose to center of robot.

        # Calibrations from GTM-1.
        # v_imu_to_B_in_IMU = -0.1251 0.0373 0.1019
        # q_IMU_B = 0.3827 0 0 -0.9239
        # v_B_to_IMU_in_B = -0.0621 0.1149 -0.1019
        # q_B_IMU = 0.3827 0 0 0.9239
        
        # IMU to Body calibration parameters.
        q_IMU_B = pyquaternion.Quaternion(0.3827, 0, 0, -0.9239)
        q_IMU_B = q_IMU_B.normalised

        p_imu_to_B_in_IMU = np.array([-0.1251,0.0373,0.1019]).reshape(3,1)

        q_G_I = pyquaternion.Quaternion(pose_msg.pose.orientation.w,
                                        pose_msg.pose.orientation.x,
                                        pose_msg.pose.orientation.y,
                                        pose_msg.pose.orientation.z)
        q_G_I = q_G_I.normalised
        p_G_I_in_G = np.array([pose_msg.pose.position.x,
                               pose_msg.pose.position.y,
                               pose_msg.pose.position.z]).reshape(3,1)

        q_G_B = q_G_I * q_IMU_B
        q_G_B = q_G_B.normalised


        p_B_in_G = p_G_I_in_G + q_G_I.rotation_matrix.dot(p_imu_to_B_in_IMU)

        transformed_pose_msg = PoseStamped()
        transformed_pose_msg.header = pose_msg.header
        transformed_pose_msg.pose.orientation.w = q_G_B[0]
        transformed_pose_msg.pose.orientation.x = q_G_B[1]
        transformed_pose_msg.pose.orientation.y = q_G_B[2]
        transformed_pose_msg.pose.orientation.z = q_G_B[3]

        transformed_pose_msg.pose.position.x = p_B_in_G[0]
        transformed_pose_msg.pose.position.y = p_B_in_G[1]
        transformed_pose_msg.pose.position.z = p_B_in_G[2]

        #R_GI, p_GI_in_G = self.poseMsgToRt(pose_msg)
        #R = SO3.from_quaternion([pose_msg.pose.orientation.w,
        #                         pose_msg.pose.orientation.x,
        #                         pose_msg.pose.orientation.y,
        #                         pose_msg.pose.orientation.z]).as_matrix()

        

        # In standalone mode, we disregard the porpagated poses from the MARS and use the solver node as a
        # standalone forwrad-kinematics pose measurement.
        #self.latest_pose_from_mars = transformed_pose_msg
        print(transformed_pose_msg.pose)
        if not self.standalone_mode:
            self.best_known_pose = transformed_pose_msg
        #pdb.set_trace()
        #print("q_G_I:     ", q_G_I.elements)
        #print("q_G_B:     ", q_G_B.elements)
        #print("p_B_in_G:  ", p_B_in_G)

    def measUpdate(self, msg):
        #pdb.set_trace()
        self.latest_cable_length = np.array(msg.length).reshape(4,1)
        self.latest_cable_length_stamp = msg.header.stamp

        #Do not attempt to run the solvers if initial pose is unknown
        if self.best_known_pose is None:
            print('self.best_known_pose is None')
            return False
        #Deactivate the solver if the last pose measuremetn is too old (more than 0.5 seconds)
        if msg.header.stamp.to_sec()-self.best_known_pose.header.stamp.to_sec() < 1:
            self.pose_recent_enough = True
        else:
            self.pose_recent_enough = True

        # In the best known pose is available and recent enough, allow the solvers to run
        if self.pose_recent_enough and self.init_length_ready:
            self.solver_ready = True
        else:
            self.solver_ready = False

        #Run the solvers only if the initial cable lengths are known and if the available pose propagations are recent enough
        if self.solver_ready:
            stamp = msg.header.stamp.to_sec()
            if (np.abs(self.last_exec_stamp - stamp) >= self.sampling_time):
                print('New length measuremtn arrived. Running the solvers!')
                best_known_rot, best_known_pos = self.poseMsgToRt(self.best_known_pose)

                #Let's simulate the situation where we get propabeted poses by purturbing the optitrack poses
                #R_rand = SO3.exp(np.random.randn(3)*0.5*np.pi/180).as_matrix()
                #best_known_rot = best_known_rot @ R_rand
                #r_rand = np.random.randn(3).reshape(3,1)*0.1
                #best_known_pos = np.array(best_known_pos) + r_rand

                #Using the propagated (perturbed) poses, run the first solver to get the initial guess for the cable forces
                R, lc_cat, cable_forces, C1, C2, b_in_w \
                = scampi_ks_solver.inverseKinematicsSolver(self.robot_params, best_known_pos, best_known_rot)
                #Correct the offset of the measured cable lengths
                cable_lenght = np.array(msg.length).reshape(4,1) + self.init_length.reshape(4,1)
                #Run the FK solver with the real cable length measurements and cable_force intial guess provided by the first solver
                fk_R, fk_T, fk_fc, fk_C1, fk_C2, fk_b_in_w\
                = scampi_ks_solver.forwardKinematicsSolver(self.robot_params,
                                                           cable_lenght,
                                                           cable_forces[:,0],
                                                           best_known_pos,
                                                           best_known_rot)

                pose_msg = self.rtToPoseMsg(fk_R, fk_T)
                msg.header.frame_id = "world"
                #The stamp of the FK result is the same as the stamp for the cable length sensors
                pose_msg.header = msg.header
                #Store the optimized bose for the next iteration if standalone mode is activated
                if self.standalone_mode:
                    print('Updating the Best Known pose')
                    self.best_known_pose = pose_msg

                self.last_exec_stamp = stamp

                #Publish the FK result if required
                if self.result_publisher is not None:
                    self.result_publisher.publish(pose_msg)

        return True

    def poseMsgToRt(self, pose_msg):
        #pdb.set_trace()
        R = SO3.from_quaternion([pose_msg.pose.orientation.w,
                                 pose_msg.pose.orientation.x,
                                 pose_msg.pose.orientation.y,
                                 pose_msg.pose.orientation.z]).as_matrix()

        T = np.array([pose_msg.pose.position.x,
                      pose_msg.pose.position.y,
                      pose_msg.pose.position.z]).reshape(3,1)
        return R,T

    def rtToPoseMsg(self, R, T):
        #pdb.set_trace()
        pose_msg = PoseStamped()
        #Normalize
        u, s, v = np.linalg.svd(R)
        R = u@v
        q = pyquaternion.Quaternion(matrix = R).elements

        pose_msg.pose.orientation.w = q[0]
        pose_msg.pose.orientation.x = q[1]
        pose_msg.pose.orientation.y = q[2]
        pose_msg.pose.orientation.z = q[3]

        pose_msg.pose.position.x = T[0]
        pose_msg.pose.position.y = T[1]
        pose_msg.pose.position.z = T[2]
        return pose_msg
