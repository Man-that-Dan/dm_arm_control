#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from threading import Event
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from dm_arm_interfaces.action import MoveArm, MoveJoint
from dm_arm_interfaces.srv import IKRequest, FKRequest
from functools import partial
from rclpy.callback_groups import ReentrantCallbackGroup


class ArmMovementServer(Node):


    def __init__(self):
        super().__init__("arm_movement")
        self._action_server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            self.move_arm_to_position)
        self.angles = [0.0, 0.0, 0.0, 0.0, 0.0] 

    def move_arm_to_position(self, goal_handle):
        self.get_logger().info('Executing goal.')
        #send the goal position to the IK service
        self.get_logger().info(str(type(self.angles[0])))
        goal_position = goal_handle.request.goal_position
        self.ik_client = self.create_client(IKRequest, 'dm_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = IKRequest.Request()
        self.req.pose = goal_position
        
        
        self.req.current_joint_angles = self.angles
        self.completed = False
        self.future = self.ik_client.call_async(self.req)
        self.ik_completed = False
        self.error_ocurred = False

        done_event = Event()
        def get_joint_positions(future):
            nonlocal done_event
            try:
                self.get_logger().info('received angles from IK Service')
                response = future.result()
                self.goal_joint_positions = response.angles
                done_event.set()
            except Exception as ex:
                self.get_logger().error("exception ocurred getting joint positions %r" % (ex,))
                self.error_ocurred = True
                done_event.set()

        self.future.add_done_callback(get_joint_positions)

        done_event.wait()
        
        if self.error_ocurred:
            goal_handle.fail()
            self.get_logger().error('goal failed')
            result = MoveArm.Result()
            result.success = False
            return result
        else:
            #send the values to the driver node
            self.get_logger().info('engaging drivers')
            done_event.clear()
            self._driver_client = ActionClient(self, MoveJoint, 'move_joints', callback_group=ReentrantCallbackGroup())
            driver_goal = MoveJoint.Goal()
            driver_goal.angle = self.goal_joint_positions
            while not self._driver_client.wait_for_server(timeout_sec=4.0):
                self.get_logger().info('waiting for driver...')
            self.driver_completed = Event()    
            #TODO figure out why feedback function just blocks forever and reincorporate it
            self._send_goal_future = self._driver_client.send_goal_async(driver_goal)
            self._send_goal_future.add_done_callback(partial(self.driver_goal_callback, driver_completed=self.driver_completed))
            
            self.driver_completed.wait()
            if self.error_ocurred:
                result = MoveArm.Result()
                result.success = False
                return result
            else:
                goal_handle.succeed()
                result = MoveArm.Result()
                result.success = True
                return result 

    # def get_joint_positions(self, future):
    #     try:
    #         self.get_logger().info('received angles from IK Service')
    #         response = future.result()
    #         self.goal_joint_positions = response.angles
    #         self.ik_completed = True
    #     except Exception as ex:
    #         self.get_logger().error("exception ocurred getting joint positions %r" % (ex,))
    #         self.ik_completed = True
    #         self.error_ocurred = True
    
    def driver_feedback_callback(self, driver_feedback, goal_handle):
        #send angles to fk service to get current position
        self.get_logger().info('got feedback from driver')
        current_positions = driver_feedback.feedback.last_angles_set
        self.angles = current_positions
        self.fk_client = self.create_client(FKRequest, 'dm_fk')
        self.fk_client.wait_for_service()
        req = FKRequest.Request()
        req.angles = current_positions
        done_event = Event()
        self.fk_response = None
        def send_feedback(future):
            nonlocal done_event
            self.get_logger().info('sending feeeeeedback')
            self.fk_response = future.result()
            done_event.set()
           

        future = self.fk_client.call_async(req)
        self.get_logger().info('awaiting fk service')
        future.add_done_callback(send_feedback)
        done_event.wait()
        try:    
            current_feedback = MoveArm.Feedback()
            current_feedback.current_position = self.fk_response.pose
            goal_handle.publish_feedback(current_feedback)
        except Exception as ex:
            self.get_logger().error("exception sending feedback")



    # def send_feedback(self, future, goal_handle):
    #     try:
    #         self.get_logger().info('sending feeeeeedback')
    #         response = future.result()
    #         current_feedback = MoveArm.Feedback()
    #         current_feedback.current_position = response.pose
    #         goal_handle.publish_feedback(current_feedback)
    #     except Exception as ex:
    #         self.get_logger().error("exception sending feedback")

    def driver_goal_callback(self, future, driver_completed):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected')
            return
        self.get_logger().info('goal accepted')
            
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(partial(self.driver_complete_callback, driver_completed=driver_completed))

    def driver_complete_callback(self, future, driver_completed):
        self.get_logger().info('driver completed')
        response = future.result().result
        self.angles = response.angles
        if not response.success:
            self.error_ocurred = True
        else:
            self.error_ocurred = False
        driver_completed.set()  

def main(args=None):
    rclpy.init(args=args)
    node = ArmMovementServer() 
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

if __name__ == "__main__":
    main()


