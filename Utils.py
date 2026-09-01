import numpy as np
from scipy.spatial.transform import Rotation

def linear_to_angular(msg):
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        roll, pitch, yaw = Rotation.from_quat(quat).as_euler('xyz', degrees=False)

        z = np.array([
            [roll],
            [pitch],
            [yaw],
            [msg.angular_velocity.x],
            [msg.angular_velocity.y],
            [msg.angular_velocity.z]
        ])

        return z