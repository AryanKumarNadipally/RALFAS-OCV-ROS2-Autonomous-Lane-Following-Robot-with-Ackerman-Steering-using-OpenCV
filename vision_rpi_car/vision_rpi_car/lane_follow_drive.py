#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import RPi.GPIO as GPIO

class AckermannDriveNode(Node):
    def __init__(self):
        super().__init__('ackermann_drive_node')
        
        self.error_value_subscriber = self.create_subscription(
            Int16, '/line_following_error', self.error_callback, 10)
        
        
        GPIO.setmode(GPIO.BCM)
        
        self.right_motor_a = 24
        self.right_motor_b = 23
        self.right_enable_motor = 25
        self.left_motor_a = 15
        self.left_motor_b = 14
        self.left_enable_motor = 4
        
        self.servo_motor = 17

        
        GPIO.setup(self.right_motor_a, GPIO.OUT)
        GPIO.setup(self.right_motor_b, GPIO.OUT)
        GPIO.setup(self.right_enable_motor, GPIO.OUT)
        GPIO.setup(self.left_motor_a, GPIO.OUT)
        GPIO.setup(self.left_motor_b, GPIO.OUT)
        GPIO.setup(self.left_enable_motor, GPIO.OUT)
        
        GPIO.setup(self.servo_motor, GPIO.OUT)

        
        self.right_dc_pwm = GPIO.PWM(self.right_enable_motor, 1000)  
        self.left_dc_pwm = GPIO.PWM(self.left_enable_motor, 1000)    
        self.servo_pwm = GPIO.PWM(self.servo_motor, 50)              

        
        self.right_dc_pwm.start(0)
        self.left_dc_pwm.start(0)
        self.servo_pwm.start(0)
        
        
        self.drive_forward()

    def drive_forward(self, speed=60):  
        
        GPIO.output(self.right_motor_a, GPIO.HIGH)
        GPIO.output(self.right_motor_b, GPIO.LOW)
        GPIO.output(self.left_motor_a, GPIO.HIGH)
        GPIO.output(self.left_motor_b, GPIO.LOW)
        
        self.right_dc_pwm.ChangeDutyCycle(speed)
        self.left_dc_pwm.ChangeDutyCycle(speed)

    def set_servo_angle(self, angle):
        
        duty = angle / 18 + 2
        self.servo_pwm.ChangeDutyCycle(duty)

    def error_callback(self, msg):
        error = msg.data
        
        max_error = 100  
        angle = (error / max_error) * 90 + 90  
        angle = max(min(angle, 135), 45)  
        self.set_servo_angle(angle)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
