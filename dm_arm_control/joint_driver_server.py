#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from dm_arm_interfaces.action import MoveJoint
from functools import partial


class JointDriverServer(Node):


    def __init__(self):
        super().__init__("joint_driver")
        self._action_server = ActionServer(
            self,
            MoveJoint,
            'move_joints',
            self.move_joints)
        self.angles = [0, 0, 0, 0, 0]
        # something likeself.move_joints(self.angles) 

    def move_joints(self, goal_handle):
        self.get_logger().info('Executing goal...')
        #send the goal position to the IK service
        goal_position = goal_handle.goal_position

        # do the joint moving and publish the feedback
        current_feedback = MoveJoint.Feedback()
        try:
            current_feedback.last_angles_set = self.angles
            goal_handle.publish_feedback(current_feedback)
        except Exception as ex:
            self.get_logger().error("exception sending feedback")    
        result = True
        return result 

def main(args=None):
    rclpy.init(args=args)
    node = JointDriverServer() 
    rclpy.spin(node)

if __name__ == "__main__":
    main()