#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from dm_arm_interfaces.action import MoveArm, MoveJoint
from dm_arm_interfaces.srv import IKRequest, FKRequest
from functools import partial


class ArmMovementServer(Node):


    def __init__(self):
        super().__init__("arm_movement")
        self._driver_client = ActionClient(self, MoveJoint, 'move_joints')
        self._action_server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            self.move_arm_to_position)
        self.angles = [0, 0, 0, 0, 0] 

    def move_arm_to_position(self, goal_handle):
        self.get_logger().info('Executing goal...')
        #send the goal position to the IK service
        goal_position = goal_handle.goal_position
        self.ik_client = self.create_client(IKRequest, 'dm_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = IKRequest.Request()
        self.req.pose = goal_position
        self.req.current_joint_angles = self.angles
        self.completed = False
        self.future = self.ik_client.call_async(self.req)
        self.future.add_done_callback(partial(self.get_joint_positions))
        while not self.ik_completed:
            self.get_logger().info('getting joint positions from kinematics service')
        if self.error_ocurred:
            goal_handle.fail()
            result = MoveArm.Result()
            result.success = False
            return result
        else:
            #send the values to the driver node
            driver_goal = MoveJoint.Goal()
            driver_goal.angles = self.goal_joint_positions
            while not self._driver_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('waiting for driver...')
            self._send_goal_future = self._driver_client.send_goal_async(driver_goal, feedback_callback=partial(self.driver_feedback_callback, goal_handle=goal_handle))
            self._send_goal_future.add_done_callback(self.driver_complete_callback)
            while not self.driver_completed:
                self.get_logger().info('waiting for driver to finish setting servos')
            if self.error_ocurred:
                goal_handle.fail()
                result = MoveArm.Result()
                result.success = False
                return result
            else:
                result = MoveArm.Result()
                result.success = True
                return result 

    def get_joint_positions(self, future):
        try:
            response = future.result()
            self.goal_joint_positions = response.angles
            self.ik_completed = True
        except Exception as ex:
            self.get_logger().error("exception ocurred getting joint positions %r" % (ex,))
            self.error_ocurred = True
    
    def driver_feedback_callback(self, driver_feedback):
        #send angles to fk service to get current position
        current_positions = driver_feedback.last_angles_set
        self.angles = current_positions
        self.fk_client = self.create_client(FKRequest, 'dm_fk')
        self.fk_client.wait_for_service()
        req = FKRequest.Request()
        req.angles = current_positions
        future = self.fk_client.call_async(req)
        future.add_done_callback(partial(self.send_feedback))
        

    def send_feedback(self, future, goal_handle):
        try:
            response = future.result()
            current_feedback = MoveArm.Feedback()
            current_feedback.current_position = response.pose
            goal_handle.publish_feedback(current_feedback)
        except Exception as ex:
            self.get_logger().error("exception sending feedback")

    def driver_complete_callback(self, future):
        return


def main(args=None):
    rclpy.init(args=args)
    node = ArmMovementServer() 
    rclpy.spin(node)

if __name__ == "__main__":
    main()


