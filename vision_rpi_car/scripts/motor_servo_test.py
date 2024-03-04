import sys
import signal
from time import sleep
import RPi.GPIO as GPIO

def signal_handler(sig, frame):
    right_dc_pwm.stop()
    left_dc_pwm.stop()
    servo_pwm.stop()
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

right_motor_a = 24
right_motor_b = 23
right_enable_motor = 25

left_motor_a = 15
left_motor_b = 14
left_enable_motor = 4

servo_motor = 17

GPIO.setmode(GPIO.BCM)

GPIO.setup(right_motor_a, GPIO.OUT)
GPIO.setup(right_motor_b, GPIO.OUT)
GPIO.setup(right_enable_motor, GPIO.OUT)
GPIO.setup(left_motor_a, GPIO.OUT)
GPIO.setup(left_motor_b, GPIO.OUT)
GPIO.setup(left_enable_motor, GPIO.OUT)
GPIO.setup(servo_motor, GPIO.OUT)

right_dc_pwm = GPIO.PWM(right_enable_motor, 1000)
right_dc_pwm.start(0)
left_dc_pwm = GPIO.PWM(left_enable_motor, 1000)
left_dc_pwm.start(0)
servo_pwm = GPIO.PWM(servo_motor, 50)
servo_pwm.start(0)

def setServoAngle(angle):
    duty =  angle / 18 + 2
    GPIO.output(servo_motor, True)
    servo_pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(servo_motor, False)
    servo_pwm.ChangeDutyCycle(duty)

def loop():

    while True:

        x = input("Which motor \nf - forward\nb - backward\ns - stop\nz - speed\ne - exit\nt - servo control: ")
        if x == 'f':
            print("\nForward\n")
            GPIO.output(right_motor_a, GPIO.HIGH)
            GPIO.output(right_motor_b, GPIO.LOW)
            GPIO.output(left_motor_a, GPIO.HIGH)
            GPIO.output(left_motor_b, GPIO.LOW)
            right_dc_pwm.ChangeDutyCycle(75)
            left_dc_pwm.ChangeDutyCycle(75)
            x = 'o'

        elif x == 'b':
            print("\nBackward\n")
            GPIO.output(right_motor_a, GPIO.LOW)
            GPIO.output(right_motor_b, GPIO.HIGH)
            GPIO.output(left_motor_a, GPIO.LOW)
            GPIO.output(left_motor_b, GPIO.HIGH)
            right_dc_pwm.ChangeDutyCycle(75)
            left_dc_pwm.ChangeDutyCycle(75)

            x = 'o'

        elif x == 's':
            print("\nStop\n")
            GPIO.output(right_motor_a, GPIO.LOW)
            GPIO.output(right_motor_b, GPIO.LOW)
            GPIO.output(left_motor_a, GPIO.LOW)
            GPIO.output(left_motor_b, GPIO.LOW)
            right_dc_pwm.ChangeDutyCycle(0)
            left_dc_pwm.ChangeDutyCycle(0)
            x = 'o'

        elif x == 'z':
            x = int(input('Input Speed 0 - 100\n'))
            GPIO.output(right_motor_a, GPIO.HIGH)
            GPIO.output(right_motor_b, GPIO.LOW)
            GPIO.output(left_motor_a, GPIO.HIGH)
            GPIO.output(left_motor_b, GPIO.LOW)
            right_dc_pwm.ChangeDutyCycle(x)
            left_dc_pwm.ChangeDutyCycle(x)
            x = 'o'

        elif x == "e":
            right_dc_pwm.stop()
            left_dc_pwm.stop()
            servo_pwm.stop()
            GPIO.cleanup()
            break

        elif x == 't':
            x =  int(input("\nInput angle\n"))
            setServoAngle(x)
            x = 'o'

        else:
            print("Wrong Input !\n\n")


if __name__ == '__main__':
    loop()