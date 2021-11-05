#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dm_arm_interfaces.srv import IKRequest, FKRequest
from geometry_msgs.msg import Pose
import numpy as np
import math


class KinematicsService(Node): 
    def __init__(self):
        super().__init__("kinematics_service")
        #link lengths
        self.declare_parameter('a1', 0.1)
        self.declare_parameter('a2', 0.6)
        self.declare_parameter('a3', 0.5)
        self.sampling_distance = 0.1
        self.learning_rate = 2
        self.server_ = self.create_service(IKRequest, "dm_ik", self.handle_ik_request)
        self.server_ = self.create_service(FKRequest, "dm_fk", self.handle_fk_request)

    def handle_ik_request(self, request, response):
        pose = request.pose
        current_joints = request.current_joint_angles
        target_position = [pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]  
        angles = self.inverse_kinematics(target_position, current_joints)
        response.angles = angles  
        return response

    def handle_fk_request(self, request, response):
        current_joints = request.angles
        current_position = self.fk_get_displacement(current_joints[0], current_joints[1], current_joints[2])
        current_orientation = self.get_orientation(current_joints)
        result_pose = Pose()
        result_pose.position = current_position
        result_pose.orientation = current_orientation
        return response    

    def distance_from_target(self, a, angles):
        b = self.fk_get_displacement(angles[0], angles[1], angles[2])
        arr = np.array(a)
        brr = np.array(b)
        dist = np.linalg.norm(arr-brr)
        return dist
        
    def fk_get_displacement(self, theta1, theta2, theta3):
        a1 = self.get_parameter('a1').get_parameter_value().float_value
        a2 = self.get_parameter('a2').get_parameter_value().float_value
        a3 = self.get_parameter('a3').get_parameter_value().float_value
        x = np.cos(theta1 * (np.pi/180)) * ((np.cos(theta2 * (np.pi/180)) * a2) + (np.cos((theta3+theta2) * (np.pi/180)) * a3))
        y = np.sin(theta1 * (np.pi/180)) * ((np.cos(theta2 * (np.pi/180)) * a2) + (np.cos((theta3+theta2) * (np.pi/180)) * a3))
        z = a1 + ((np.sin(theta2 * (np.pi/180)) * a2) + (np.sin((theta3+theta2) * (np.pi/180)) * a3))
        return [x, y, z]

    def partial_gradient (self, target, angles, i):
        angle = angles[i]
        #Gradient : [F(x+SamplingDistance) - F(x)] / h
        f_x = self.distance_from_target(target, angles)
        angles[i] += self.sampling_distance
        f_x_plus_d = self.distance_from_target(target, angles)
        gradient = (f_x_plus_d - f_x) / self.sampling_distance
        #Restores
        angles[i] = angle
        return gradient

    def inverse_kinematics (self, target, angles):
        target_pos = [target[0], target[1], target[2]]
        manipulator_angles = [angles[0], angles[1], angles[2]]
        while self.distance_from_target(target_pos, manipulator_angles) > 0.003:
            i = 0
            for angle in manipulator_angles:
                # Gradient descent
                # Update : Solution -= LearningRate * Gradient
                gradient = self.partial_gradient(target_pos, manipulator_angles, i)
                manipulator_angles[i] -= self.learning_rate * gradient
                i += 1
        angles[0] = round(manipulator_angles[0],1)
        angles[1] = round(manipulator_angles[1],1)
        angles[2] = round(manipulator_angles[2],1)
        target_orientation = [target[3], target[4], target[5], target[6]] 
        end_effector_angles = self.get_effector_angles(target_orientation, angles)
        angles[3] = round(end_effector_angles[0],1)
        angles[4] = round(end_effector_angles[1],1)
        return angles  

    def get_orientation(self, angles):
        #quaternion representing orientation of end effector at all joints 0 degrees
        base_orientation = [0.49985,0.49985,0.49985,0.49985]

        #quaternion representing orientation of end effector after  1st joint set
        pre_theta5_orientation = self.quaternion_multiply([np.cos((angles[0] /2) * (np.pi/180)), 0, np.sin((angles[0] / 2) * (np.pi/180)), 0], base_orientation)
        
        #quaternion representing orientation of end effecter after joints 1 - 4 set
        pre_theta5_orientation = self.quaternion_multiply([np.cos((angles[3] / 2) * (np.pi/180)), 0, np.sin((angles[3] / 2) * (np.pi/180)), 0], pre_theta5_orientation)

        final_orientation = self.quaternion_multiply([np.cos((angles[4] / 2) * (np.pi/180)), 0, 0, np.sin((angles[3] / 2) * (np.pi/180))], pre_theta5_orientation)

        return final_orientation   

    def get_effector_angles(self, target_orientation, angles):
        rot_matrix_6_0 = self.quaternion_rotation_matrix(target_orientation)
        z_6_0_projection = rot_matrix_6_0[2][2]
    
        #get theta4 joint position based on projection from end effector z to base frame z
        theta4 = round((np.arcsin(z_6_0_projection) / (np.pi/180)) - angles[1] - angles[2], 0)

        #quaternion representing orientation of end effector at all joints 0 degrees
        base_orientation = [0.49985,0.49985,0.49985,0.49985]

        #quaternion representing orientation of end effector after  joint set
        pre_theta5_orientation = self.quaternion_multiply([np.cos((angles[0] /2) * (np.pi/180)), 0, np.sin((angles[0] / 2) * (np.pi/180)), 0], base_orientation)
        
        #quaternion representing orientation of end effecter after joints 1 - 4 set
        pre_theta5_orientation = self.quaternion_multiply([np.cos((theta4 / 2) * (np.pi/180)), 0, np.sin((theta4 / 2) * (np.pi/180)), 0], pre_theta5_orientation)

        #inverse of orientation before joint 5
        inv_pre_theta5 = [pre_theta5_orientation[0], -pre_theta5_orientation[1], -pre_theta5_orientation[2], -pre_theta5_orientation[3]]

        #quaternion representing difference between target orientation and pre-5th joint orientation
        diff = self.quaternion_multiply(inv_pre_theta5, target_orientation)
        
        #rotation matrix representing difference
        diff_rot = self.quaternion_rotation_matrix(diff)
        
        #angle for joint 5
        theta5 = math.degrees(np.arctan(diff_rot[1][0] / diff_rot[0][0]))
        
        return [theta4, theta5]
        
    #dot product of two quaternions    
    def quaternion_dot(self, left, right):
        return left[1] * right[1] + left[2] * right[2] + left[3] * right[3] + left[0] * right[0]

    #multiply two quaternions
    def quaternion_multiply(quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)
        
    #get rotation matrix from quaternion   
    def quaternion_rotation_matrix(Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix    

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()