import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import PhaseEnableMotor

class Motor_node(Node):
    def __init__(self):
        super().__init__('motor_node')
    
        self.right_motor = PhaseEnableMotor(phase=23, enable=18)
        self.left_motor = PhaseEnableMotor(phase=6, enable=19)

        self.latest_command = False

        self.timer = self.create_timer(0.1,self.motor_control_loop)

        self.sub = self.create_subscription(Bool,'/motor_go',self.cmd_callback,10)

    def cmd_callback(self,msg):
        self.latest_command = msg.data

    def motor_control_loop(self):
        if self.latest_command == True:
            self.right_motor.backward(1)
            self.left_motor.backward(1)
        else:
            self.right_motor.stop()
            self.left_motor.stop()

def main(args=None):
    rclpy.init(args=args)
    motor_control = Motor_node()
    try:
        rclpy.spin(motor_control)
    except KeyboardInterrupt:
        if rclpy.ok():
            motor_control.get_logger().info('Keyboard Interrupt (SIGINT) detected. Shutting down...')
    finally:
        if rclpy.ok():
            motor_control.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
