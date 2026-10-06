import rclpy
from sensor_msgs.msg import Imu
from rclpy.node import Node
import random
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

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
        self.timer = self.create_timer(0.02, self.timer_callback)
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

            t = self.get_clock().now().nanoseconds / 1e9


            roll = math.pi * math.sin(2 * math.pi * 0.05 * t)
            pitch = math.pi * math.sin(2 * math.pi * 0.05 * t)
            yaw = math.pi * math.sin(2 * math.pi * 0.05 * t)
            w_x = 0.3 * (2 * math.pi * 0.2) * math.cos(2 * math.pi * 0.05 * t)
            w_y = 0.3 * (2 * math.pi * 0.2) * math.cos(2 * math.pi * 0.04 * t)
            w_z = 0.3 * (2 * math.pi * 0.2) * math.cos(2 * math.pi * 0.06 * t)


            noisy_roll = roll + np.random.normal(0, 0.02)
            noisy_w_x = w_x + np.random.normal(0, 0.005)
            noisy_pitch = pitch + np.random.normal(0, 0.02)
            noisy_w_y = w_y + np.random.normal(0, 0.005)
            noisy_yaw = yaw + np.random.normal(0, 0.02)
            noisy_w_z = w_z + np.random.normal(0, 0.005)
            

            r = R.from_euler('xyz', [noisy_roll, noisy_pitch, noisy_yaw])
            quat = r.as_quat()

            msg.orientation.x = quat[0]
            msg.orientation.y = quat[1]
            msg.orientation.z = quat[2]
            msg.orientation.w = quat[3]

            msg.angular_velocity.x = noisy_w_x
            msg.angular_velocity.y = noisy_w_y
            msg.angular_velocity.z = noisy_w_z

            pub.publish(msg)
            self.get_logger().info(
                f"IMU{i} data:\n"
                f"orient: [{msg.orientation.x:.2f}, {msg.orientation.y:.2f}, {msg.orientation.z:.2f}, {msg.orientation.w:.2f}],\n"
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

        
