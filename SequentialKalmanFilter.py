import numpy as np

def wrap_angle(angle):
    # Maps an angle or vector of angles to [-pi, pi]
    return np.arctan2(np.sin(angle), np.cos(angle))

class SequentialKalmanFilter:
    def __init__(self, dim_z, dim_x):
        self.dim_z = dim_z
        self.dim_x = dim_x

        ang_err = 1e-4
        vel_err = 1e-2
        self.x = np.zeros((dim_x, 1))
        self.P = np.eye(dim_x)
        self.Q = np.diag([ang_err, ang_err, ang_err, vel_err, vel_err, vel_err])
        self.F = np.eye(6)
        self.H = np.eye(6)

        self.consecutive_rejects = 0
        self.max_rejects = 10
        self.d_last = 0.0


    def predict(self, dt):
        # State transition update
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        # Predicted state and covariance matrix
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        self.x[0:3] = wrap_angle(self.x[0:3])

    def update(self, z, R):
        # Innovation
        self.y = z - self.H @ self.x

        # Wrapping angles for roll, pitch and yaw
        self.y[0:3] = wrap_angle(self.y[0:3])

        # Innovation covariance matrix
        self.S = self.H @ self.P @ self.H.T + R

        # Test if measurement is correct. Mahalanobis test
        d = (self.y.T @ np.linalg.inv(self.S).T @ self.y).item()
        self.d_last = d
        if d > 16.81:
            self.consecutive_rejects += 1
            self.P *= 1.15

            # if measurement has been rejected more than 10 times it resets
            if self.consecutive_rejects >= self.max_rejects:
                self.x = z.copy()
                self.P = np.eye(self.dim_x) * 0.1
                self.consecutive_rejects = 0
                return True

            return False

        self.consecutive_rejects = 0

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(self.S)

        # State update
        self.x = self.x + K @ self.y

        # Wrapping angles in case of multiple rotations
        self.x[0:3] = wrap_angle(self.x[0:3])

        # Covariance update
        IKH = (np.eye(self.dim_x) - K @ self.H)
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

        return True

