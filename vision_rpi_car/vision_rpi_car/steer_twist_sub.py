#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class AckermannSteeringNode(Node):
    def __init__(self):
        super().__init__('ackermann_steering_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        self.right_motor_a = 24
        self.right_motor_b = 23
        self.right_enable_motor = 25

        self.left_motor_a = 15
        self.left_motor_b = 14
        self.left_enable_motor = 4

        self.servo_motor = 17  # Steering servo

        GPIO.setup(self.right_motor_a, GPIO.OUT)
        GPIO.setup(self.right_motor_b, GPIO.OUT)
        GPIO.setup(self.right_enable_motor, GPIO.OUT)
        GPIO.setup(self.left_motor_a, GPIO.OUT)
        GPIO.setup(self.left_motor_b, GPIO.OUT)
        GPIO.setup(self.left_enable_motor, GPIO.OUT)
        GPIO.setup(self.servo_motor, GPIO.OUT)

        # Initialize PWM
        self.right_dc_pwm = GPIO.PWM(self.right_enable_motor, 1000)  # 1000 Hz for motor
        self.left_dc_pwm = GPIO.PWM(self.left_enable_motor, 1000)    # 1000 Hz for motor
        self.servo_pwm = GPIO.PWM(self.servo_motor, 50)              # 50 Hz for servo

        self.right_dc_pwm.start(0)
        self.left_dc_pwm.start(0)
        self.servo_pwm.start(0)

    def set_motor_speed(self, direction, speed_percentage):
        if direction == 'forward':
            GPIO.output(self.right_motor_a, GPIO.HIGH)
            GPIO.output(self.right_motor_b, GPIO.LOW)
            GPIO.output(self.left_motor_a, GPIO.HIGH)
            GPIO.output(self.left_motor_b, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(self.right_motor_a, GPIO.LOW)
            GPIO.output(self.right_motor_b, GPIO.HIGH)
            GPIO.output(self.left_motor_a, GPIO.LOW)
            GPIO.output(self.left_motor_b, GPIO.HIGH)

        self.right_dc_pwm.ChangeDutyCycle(speed_percentage)
        self.left_dc_pwm.ChangeDutyCycle(speed_percentage)

    def set_servo_angle(self, angle):
        duty = angle / 18 + 2
        self.servo_pwm.ChangeDutyCycle(duty)

    def cmd_vel_callback(self, msg):
        # Linear velocity controls speed and direction
        speed = msg.linear.x
        direction = 'forward' if speed >= 0 else 'backward'
        speed_percentage = abs(speed) * 100  # Scale as needed

        # Angular velocity controls steering
        max_angular_velocity = 2.0  # This value might need tuning
        if msg.angular.z == 0:
            angle = 90  # Straight forward
        elif msg.angular.z > 0:
            # Turning right
            angle_ratio = min(msg.angular.z / max_angular_velocity, 1)
            angle = 90 + (angle_ratio * (135 - 90))  # Scale angle between 90 and 135
        else:
            # Turning left
            angle_ratio = min(abs(msg.angular.z) / max_angular_velocity, 1)
            angle = 90 - (angle_ratio * (90 - 45))  # Scale angle between 45 and 90

        self.set_motor_speed(direction, min(speed_percentage, 100))
        self.set_servo_angle(angle)

def main(args=None):
    rclpy.init(args=args)
    ackermann_steering_node = AckermannSteeringNode()
    rclpy.spin(ackermann_steering_node)
    ackermann_steering_node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
