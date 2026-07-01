import rclpy
from sensor_msgs.msg import Imu
import random


class FakeIMUPublisher(rclpy.node):
    def __init__(self):
        super().__init__("imu_subscriber")
        self.publisher_ = self.create_publisher(Imu, "/imu/data", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Fake IMU Publisher zostal uruchomiony.")
    
    def timer_callback(self):
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "fake_imu"

        msg.orientation.x = random.uniform(-1.0, 1.0)
        msg.orientation.y = random.uniform(-1.0, 1.0)
        msg.orientation.z = random.uniform(-1.0, 1.0)
        msg.orientation.w = random.uniform(-1.0, 1.0)

        msg.angular_velocity.x = random.uniform(-0.5, 0.5)
        msg.angular_velocity.y = random.uniform(-0.5, 0.5)
        msg.angular_velocity.z = random.uniform(-0.5, 0.5)
        
        msg.linear_acceleration.x = random.uniform(0, 2.0)
        msg.linear_acceleration.y = random.uniform(0, 2.0)
        msg.linear_acceleration.z = random.uniform(9.5, 10)

        self.publisher_.publish(msg)
        self.get_logger().info("Fake IMU data")


        