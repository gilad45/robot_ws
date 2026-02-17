import rclpy
from rclpy.node import Node
from gpiozero import DistanceSensor
from std_msgs.msg import Float32

class DistanceSensorNode(Node):
    def __init__(self):
        super().__init__('distance_sensor_node')

        self.sensor = DistanceSensor(echo=22, trigger=27, max_distance=4)

        self.publisher_ = self.create_publisher(Float32, '/ultrasonic/distance', 10)

        self.timer = self.create_timer(0.1 ,self.publish_distance)

        self.get_logger().info('Distance Sensor Node has started.')

    def publish_distance(self):
        msg = Float32()
        msg.data = self.sensor.distance * 100
        self.publisher_.publish(msg)
    
def main(args=None):
     rclpy.init(args=args)
     distance_node = DistanceSensorNode()
     try:
         rclpy.spin(distance_node)
     except KeyboardInterrupt:
        distance_node.get_logger().info('node stopping...')
     finally:
        if rclpy.ok():
            distance_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

