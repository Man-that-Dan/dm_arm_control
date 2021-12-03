#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from dm_arm_interfaces.action import MoveJoint
from functools import partial
import threading
from gpiozero import Device, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

Device.pin_factory = PiGPIOFactory()

class JointDriverServer(Node):


    def __init__(self):
        super().__init__("joint_driver")
        self._action_server = ActionServer(
            self,
            MoveJoint,
            'move_joints',
            self.move_joints)
        self.angles = [0.0, 0.0, 0.0, 0.0, 0.0]

        # declare parameters

        #angles that actually represent 0 for joints because the servos are not great
        #thought about parameterizing this but all of this driver node is so specific to this one arm anyways
        midAngle0 = -28.0
        midAngle1 = 0.0
        midAngle2 = -5.0
        midAngle3 = 20.0
        midAngle4 = 0.0
        self.mids = [midAngle0, midAngle1, midAngle2, midAngle3, midAngle4]

        #pins for servos
        self.declare_parameter('servo0Pin', 27)
        self.declare_parameter('servo1Pin', 22)
        self.declare_parameter('servo2Pin', 17)
        self.declare_parameter('servo3Pin', 2)
        self.declare_parameter('servo4Pin', 3)
        self.declare_parameter('endEffectorServoPin', 4)

        #speed of movement, period to move servos 1 degree
        self.declare_parameter('movement_period', 0.2)

        self.servo0 = AngularServo(self.get_parameter('servo0Pin').get_parameter_value().integer_value, max_angle=141.76035, min_angle=-141.76035, min_pulse_width = 0.0005, max_pulse_width = 0.0025, frame_width=0.020)
        self.servo1 = AngularServo(self.get_parameter('servo1Pin').get_parameter_value().integer_value, max_angle=90, min_angle=-90, min_pulse_width = 0.0005, max_pulse_width = 0.0025, frame_width=0.020)
        self.servo2 = AngularServo(self.get_parameter('servo2Pin').get_parameter_value().integer_value, max_angle=141.76035, min_angle=-141.76035, min_pulse_width = 0.0005, max_pulse_width = 0.0025, frame_width=0.020)
        self.servo3 = AngularServo(self.get_parameter('servo3Pin').get_parameter_value().integer_value, max_angle=141.76035, min_angle=-141.76035, min_pulse_width = 0.0005, max_pulse_width = 0.0025, frame_width=0.020)
        self.servo4 = AngularServo(self.get_parameter('servo4Pin').get_parameter_value().integer_value, max_angle=90, min_angle=-90, min_pulse_width = 0.0005, max_pulse_width = 0.0025, frame_width=0.020)
        self.endEffectorServo = AngularServo(self.get_parameter('endEffectorServoPin').get_parameter_value().integer_value, max_angle=45, min_angle=-45, min_pulse_width = 0.0005, max_pulse_width = 0.002, frame_width=0.020)
        self.servos = [self.servo0, self.servo1, self.servo2, self.servo3, self.servo4, self.endEffectorServo]

        #set servos to initial angles
        index = 0
        servoThreads = []
        result_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        for angle in self.angles:
            real_angle = self.translate_angle_to_real(angle, index)
            currentThread = threading.Thread(target=self.move_servo, args=(self.servos[index], real_angle, result_angles[index]))
            servoThreads.append(currentThread)
            index += 1
        for thread in servoThreads:
            thread.start()
        for thread in servoThreads:
            thread.join()
        index = 0
        for result in result_angles:
            self.angles[index] = self.translate_real_angle_to_kinematic(result, index)        
        

    def move_joints(self, goal_handle):
        self.get_logger().info('Actuating servos')
        #send the goal position to the IK service
        goal_angles = goal_handle.request.angle

        #set servos to angles
        index = 0
        servoThreads = []
        result_angles = self.angles
        for angle in goal_angles:
            real_angle = self.translate_angle_to_real(angle, index)
            currentThread = threading.Thread(target=self.move_servo, args=(self.servos[index], real_angle, result_angles[index]))
            servoThreads.append(currentThread)
            index += 1
        self.error_ocurred = False
        for thread in servoThreads:
            thread.start()
        all_done = False
        while not all_done:
            all_done = True
            for thread in servoThreads:
                if thread.is_alive():
                    all_done = False
            # publish the feedback
            current_feedback = MoveJoint.Feedback() 
            try:
                #self.get_logger().info('sending feedback')
                current_feedback.last_angles_set = result_angles
                goal_handle.publish_feedback(current_feedback)
            except Exception as ex:
                self.get_logger().error("exception sending feedback %r" % (ex,))  
        self.get_logger().info('all done')       
        for thread in servoThreads:
            thread.join()
        index = 0
        for result in result_angles:
            self.angles[index] = self.translate_real_angle_to_kinematic(result, index)        

        
         
        result = MoveJoint.Result()
        if not self.error_ocurred:
            result.success = True
            goal_handle.succeed()
        else: 
            result.success = False
        result.angles = self.angles
        return result

    def move_servo(self, servo, new_angle, set_angle):
        movement_period = self.get_parameter('movement_period').value
        self.get_logger().info('moving servo')
        try:
            while not round(servo.angle, 0) == round(new_angle, 0):
                self.get_logger().info('servo angle ' + str(servo.angle) + ' target ' + str(new_angle))
                if new_angle > servo.angle:
                    servo.angle += 1
                    sleep(movement_period)
                elif new_angle < servo.angle:
                    servo.angle -= 1
                    sleep(movement_period)
        except:
            self.error_ocurred = True
        set_angle = servo.angle

    def translate_angle_to_real(self, angle, servoIndex):
        actualZero = self.mids[servoIndex]
        return actualZero + angle

    def translate_real_angle_to_kinematic(self, angle, servoIndex):
        actualZero = self.mids[servoIndex]
        return angle - actualZero                     

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = JointDriverServer() 
    rclpy.spin(node, executor)

if __name__ == "__main__":
    main()