#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# GPIO Pin Setup for L2983N Motor Driver
LEFT_MOTOR_PIN1 = 17  # IN1 (Forward for left motor)
LEFT_MOTOR_PIN2 = 27  # IN2 (Backward for left motor)
RIGHT_MOTOR_PIN1 = 23  # IN3 (Forward for right motor)
RIGHT_MOTOR_PIN2 = 24  # IN4 (Backward for right motor)

# Initialize GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_MOTOR_PIN1, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_PIN2, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_PIN1, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_PIN2, GPIO.OUT)

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Convert linear and angular velocity to motor signals
        self.process_movement(linear_x, angular_z)

    def process_movement(self, linear_x, angular_z):
        if linear_x > 0:  # Forward
            self.set_motor_high_low(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, "FORWARD")
            self.set_motor_high_low(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, "FORWARD")
        elif linear_x < 0:  # Backward
            self.set_motor_high_low(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, "BACKWARD")
            self.set_motor_high_low(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, "BACKWARD")
        elif angular_z > 0:  # Turn Left
            self.set_motor_high_low(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, "BACKWARD")
            self.set_motor_high_low(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, "FORWARD")
        elif angular_z < 0:  # Turn Right
            self.set_motor_high_low(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, "FORWARD")
            self.set_motor_high_low(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, "BACKWARD")
        else:  # Stop
            self.stop_motors()

    def set_motor_high_low(self, pin1, pin2, direction):
        if direction == "FORWARD":
            GPIO.output(pin1, GPIO.HIGH)
            GPIO.output(pin2, GPIO.LOW)
        elif direction == "BACKWARD":
            GPIO.output(pin1, GPIO.LOW)
            GPIO.output(pin2, GPIO.HIGH)

    def stop_motors(self):
        GPIO.output(LEFT_MOTOR_PIN1, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_PIN2, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_PIN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_PIN2, GPIO.LOW)

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControl()

    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        pass

    motor_control_node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()