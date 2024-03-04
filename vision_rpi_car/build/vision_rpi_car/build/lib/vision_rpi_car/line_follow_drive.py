#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import RPi.GPIO as GPIO

class AckermannDriveNode(Node):
    def __init__(self):
        super().__init__('ackermann_drive_node')
        # Subscribe to the line following error topic
        self.error_value_subscriber = self.create_subscription(
            Int16, '/line_following_error', self.error_callback, 10)
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        # Motor GPIO pins
        self.right_motor_a = 24
        self.right_motor_b = 23
        self.right_enable_motor = 25
        self.left_motor_a = 15
        self.left_motor_b = 14
        self.left_enable_motor = 4
        # Servo motor GPIO pin
        self.servo_motor = 17

        # Setup GPIO pins for motors
        GPIO.setup(self.right_motor_a, GPIO.OUT)
        GPIO.setup(self.right_motor_b, GPIO.OUT)
        GPIO.setup(self.right_enable_motor, GPIO.OUT)
        GPIO.setup(self.left_motor_a, GPIO.OUT)
        GPIO.setup(self.left_motor_b, GPIO.OUT)
        GPIO.setup(self.left_enable_motor, GPIO.OUT)
        # Setup GPIO pin for servo
        GPIO.setup(self.servo_motor, GPIO.OUT)

        # Initialize PWM for motors and servo
        self.right_dc_pwm = GPIO.PWM(self.right_enable_motor, 1000)  # 1000 Hz for DC motors
        self.left_dc_pwm = GPIO.PWM(self.left_enable_motor, 1000)    # 1000 Hz for DC motors
        self.servo_pwm = GPIO.PWM(self.servo_motor, 50)              # 50 Hz for servo

        # Start PWM with 0 duty cycle
        self.right_dc_pwm.start(0)
        self.left_dc_pwm.start(0)
        self.servo_pwm.start(0)
        
        # Optionally, set a default forward speed
        self.drive_forward()

    def drive_forward(self, speed=50):  # Speed as a percentage of maximum
        # Set both motors to move forward
        GPIO.output(self.right_motor_a, GPIO.HIGH)
        GPIO.output(self.right_motor_b, GPIO.LOW)
        GPIO.output(self.left_motor_a, GPIO.HIGH)
        GPIO.output(self.left_motor_b, GPIO.LOW)
        # Apply the speed
        self.right_dc_pwm.ChangeDutyCycle(speed)
        self.left_dc_pwm.ChangeDutyCycle(speed)

    def set_servo_angle(self, angle):
        # Convert angle to duty cycle
        duty = angle / 18 + 2
        self.servo_pwm.ChangeDutyCycle(duty)

    def error_callback(self, msg):
        error = msg.data
        # Map the error value to a servo angle
        max_error = 100  # Example, adjust based on your error range
        angle = (error / max_error) * 90 + 90  # Convert error to angle
        angle = max(min(angle, 135), 45)  # Clamp angle to servo's range
        self.set_servo_angle(angle)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveNode()
    rclpy.spin(node)
    # Cleanup and shutdown
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
