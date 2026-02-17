import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PhaseEnableMotor
from signal import SIGINT, SIGTERM, signal

# ==== GPIO PINS (BCM) ====
# Matches your previous setup
LEFT_PWM_PIN = 18
LEFT_DIR_PIN = 23
RIGHT_PWM_PIN = 19
RIGHT_DIR_PIN = 6

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # PhaseEnableMotor(phase, enable) 
        # phase = Direction, enable = PWM/Speed
        self.left_motor = PhaseEnableMotor(phase=LEFT_DIR_PIN, enable=LEFT_PWM_PIN)
        self.right_motor = PhaseEnableMotor(phase=RIGHT_DIR_PIN, enable=RIGHT_PWM_PIN)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        
        self.get_logger().info("Motor Driver Node Online (Using GPIO Zero)")

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive math
        left_speed = linear - angular
        right_speed = linear + angular

        # Clamp and move left motor
        self.move_motor(self.left_motor, left_speed)
        # Clamp and move right motor
        self.move_motor(self.right_motor, right_speed)

    def move_motor(self, motor_obj, speed):
        # Ensure speed is between -1.0 and 1.0
        speed = max(min(speed, 1.0), -1.0)
        
        if speed > 0:
            motor_obj.forward(speed)
        elif speed < 0:
            motor_obj.backward(abs(speed))
        else:
            motor_obj.stop()

    def stop_motors(self):
        self.get_logger().info("Stopping motors and releasing GPIO...")
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_motor.close()
        self.right_motor.close()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()

    try:
        # This keeps the node running and listening for messages
        rclpy.spin(node)
    except KeyboardInterrupt:
        # This catches Ctrl+C
        pass
    finally:
        # THIS IS THE FIX: This code runs no matter how the script ends
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()