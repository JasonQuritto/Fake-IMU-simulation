import numpy as np

class SequentialKalmanFilter:
    def __init__(self, dim_z, dim_x):
        self.dim_z = dim_z
        self.dim_x = dim_x

        self.x = np.zeros((dim_x, 1))
        self.P = np.eye(dim_x)
        self.Q = np.eye(dim_x)
        self.F = np.eye(6)
        self.H = np.eye(6)


    def predict(self, dt):
        # State transition update
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        # Predicted state and covariance matrix
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q


    def update(self, z, R):
        # Innovation
        self.y = z - self.H @ self.x
        # Innovation covariance matrix
        self.S = self.H @ self.P @ self.H.T + R

        # Test if measurement is correct. Mahalanobis test
        d = self.y.T @ np.linalg.inv(self.S).T @ self.y
        if d > 9.21:
            return False
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(self.S)

        # State update
        self.x = self.x + K @ self.y
        # Covariance update
        IKH = (np.eye(self.dim_x) - K @ self.H)
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

        return True

