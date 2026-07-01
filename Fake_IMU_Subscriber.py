import rclpy
from sensor_msgs.msg import Imu
from rclpy.node import Node

class FakeIMUSubscriber(Node):
    def __init__(self):
        super().__init__("imu subscriber")
        self.subscription = self.create_subscription(Imu, "/imu/data", self.listener_callback, 10)
        self.get_logger().info("IMU Subsciber got activated.")
    
    def listener_callback(self, msg):
        accel = msg.linear_acceleration
        gyro = msg.angular_velocity
        self.get_logger().info(
            f"Received: "
            f"Acceleration: [{accel.x:.2f}, {accel.y:.2f}, {accel.z:.2f}]"
            f"Gyro: [{gyro.x:.2f}, {gyro.y:.2f}, {gyro.z:.2f}]")
        
    def main(args=None):
        rclpy.init(args=args)
        node = FakeIMUSubscriber()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    
    if __name__ == "__main__":
        main()