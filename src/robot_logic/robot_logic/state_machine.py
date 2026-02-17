import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('State_machine_node')

        self.current_distance = 100
        self.should_motor_run = False
        
        self.distance = self.create_subscription(Float32, '/ultrasonic', self.distance_callback, 10)

        self.motor_drive = self.create_publisher(Bool,'/motor_go',10)

        self.timer = self.create_timer(0.1 ,self.desicion_logic)

        self.get_logger().info('state machine has started')
        

    def distance_callback(self,msg):
        self.current_distance = msg.data

    def desicion_logic(self):
        if self.current_distance < 20.0:
            self.should_motor_run = False
        else:
            self.should_motor_run = True
        
        msg = Bool()
        msg.data = self.should_motor_run
        self.motor_drive.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    state_node = StateMachineNode()
    try:
        rclpy.spin(state_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            state_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()



