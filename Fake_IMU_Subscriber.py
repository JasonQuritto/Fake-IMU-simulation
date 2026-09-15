import rclpy
from sensor_msgs.msg import Imu
from rclpy.node import Node
from SequentialKalmanFilter import SequentialKalmanFilter
import numpy as np
from Utils import linear_to_angular

class MultiIMUFusionSubscriber(Node):

    def __init__(self, n_imus):
        super().__init__("Multi_IMU_Fusion_Subscriber")
        # Kalman filter initialization
        self.KF = SequentialKalmanFilter(dim_z = 6, dim_x = 6)

        self.subscribers_list = []

        for i in range(n_imus):
            callback = lambda msg, imu_id = i: self.listener_callback(msg, imu_id)
            
            subscriber = self.create_subscription(Imu, f"/imu{i}/data", callback, 10)
            self.subscribers_list.append(subscriber)

        # Prediction timer
        self.dt = 0.02
        self.last_update_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(self.dt, self.timer_predict_callback)

        # Measurement Error matrix
        var_angle = 0.01
        var_gyro = 0.000025
        self.R = np.diag([var_angle, var_angle, var_angle, var_gyro, var_gyro, var_gyro])

        self.get_logger().info("IMU Subsciber got activated.")


    def timer_predict_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self.dt = now - self.last_update_time
        self.last_update_time = now

        if self.dt <= 0.0 or self.dt > 0.5:
            return
        
        self.KF.predict(self.dt)

    
    def listener_callback(self, msg, imu_id):

        z = linear_to_angular(msg)

        # Checking if measurement passed the test
        accepted = self.KF.update(z, self.R)

        # State update
        if accepted:
            self.get_logger().info(
                f"\n[ESTYMACJA KALMANA [IMU:{imu_id}]]\n"
                f"Roll:  {self.KF.x[0, 0]:6.3f} rad | Pitch: {self.KF.x[1, 0]:6.3f} rad | Yaw: {self.KF.x[2, 0]:6.3f} rad\n"
                f"Gyro:  wx={self.KF.x[3, 0]:5.2f} | wy={self.KF.x[4, 0]:5.2f} | wz={self.KF.x[5, 0]:5.2f}\n"
)
        else:
            self.get_logger().warn(
                f"IMU{imu_id} rejected | d = {self.KF.d_last:.2f} | "
                f"y_yaw = {self.KF.y[2, 0]:.3f} | x_yaw = {self.KF.x[2, 0]:.3f}"
            )


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
