import rclpy
from sensor_msgs.msg import Imu
from rclpy.node import Node
import SequentialKalmanFilter
import numpy as np
from Utils import linear_to_angular

class MultiIMUFusionSubscriber(Node):
    def __init__(self, n_imus):
        super().__init__("Multi IMU Fusion Subscriber")
        # Kalman filter initialization
        self.KF = SequentialKalmanFilter(dim_z = 1, dim_x = 2)

        self.subscribers_list = []

        for i in range(n_imus):
            callback = lambda msg, imu_id = i: self.listener_callback(msg, imu_id)
            
            subscriber = self.create_subscription(Imu, f"/imu{i}/data", callback, 10)
            self.subscribers_list.append(subscriber)

        # Prediction timer
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.timer_predict_callback)

        # Measurement Error matrix
        self.R = np.array([[0.02]])

        self.get_logger().info("IMU Subsciber got activated.")


    def timer_predict_callback(self):
        self.KF.predict(self.dt)
    
    def listener_callback(self, msg, imu_id):

        z = self.linear_to_angular(msg)

        # Checking if measurement passed the test
        accepted = self.KF.update(z, self.R)

        # State update
        if accepted:
            fused_angle = self.KF.x[0, 0]
            fused_velocity = self.KF.x[1, 0]
            self.get_logger().info(
                f"[IMU {imu_id}] Angle: {fused_angle:.3f} | Angular Velocity: {fused_velocity:.3f}"
            )
        else:
            self.get_logger().warn(f"IMU{imu_id} reading rejected by test.")


def main(args=None):
    rclpy.init(args=args)
    node = MultiIMUFusionSubscriber(n_imus = 2)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()