import rclpy
from sensor_msgs.msg import Imu
from rclpy.node import Node
import random


class MultiIMUPublisher(Node):


    def __init__(self, n_imus=2):
        super().__init__("Multi_imu_publisher")
        self.n_imus = n_imus
        self.publishers_list = []

        # creating a list of sensors for n_imus publishers
        for i in range(n_imus):
            pub = self.create_publisher(Imu, f"/imu{i}/data", 10)
            self.publishers_list.append(pub)

        # callback which makes all sensors publicate their data on different topics
        self.timer = self.create_timer(1, self.timer_callback)
        self.get_logger().info("Multi IMU Publisher zostal uruchomiony.")
    
    def timer_callback(self):
        for i, pub in enumerate(self.publishers_list):

            # malfunction simulation
            if random.random() <= 0.05:
                self.get_logger().warn(
                    f"[MALFUNCTION] Sensor /imu{i}/data has lost a packet."
                )
                continue
            
            # generating data
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
            self.get_logger().info(f"IMU data:\n"
                                f"orient: [{msg.orientation.x:.2f}, {msg.orientation.y:.2f}, {msg.orientation.z:.2f}, {msg.orientation.w:.2f}],\n"
                                f"accel: [{msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}],\n"
                                f"gyro: [{msg.angular_velocity.x:.2f}, {msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f}]\n"
                )
        

def main(args=None):
    rclpy.init(args=args)
    node = MultiIMUPublisher(n_imus = 2)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()

        