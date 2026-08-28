import numpy as np

class SequentialKalmanFilter:
    def __init__(self, dim_z, dim_x):
        self.dim_z = dim_z
        self.dim_x = dim_x

        self.x = np.zeros((dim_x, 1))
        self.P = np.eye(dim_x)
        self.Q = np.eye(dim_x)


    def predict(self, dt):
        # State transition matrix
        F = np.array([[1, dt], [0, 1]])

        # Predicted state and covariance matrix
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q


    def update(self, z, R):
        # Observation matrix
        H = np.array([[0, 1]])

        # Innovation
        self.y = z - H @ self.x
        # Innovation covariance matrix
        self.S = H @ self.P @ H.T + R

        # Test if measurement is correct. Mahalanobis test
        d = self.y.T @ np.linalg.inv(self.S).T @ self.y
        if d > 9.21:
            return False
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(self.S)

        # State update
        self.x = self.x + K @ self.y
        # Covariance update
        IKH = (np.eye(self.dim_x) - K @ H)
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

        return True

