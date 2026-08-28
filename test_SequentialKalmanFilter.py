from SequentialKalmanFilter import SequentialKalmanFilter
import numpy as np


def test_KF_init():
    KF = SequentialKalmanFilter(1, 2)
    assert KF.dim_x == 2
    assert KF.dim_z == 1
    assert KF.x.shape == (2, 1)
    assert KF.P.shape == (2, 2)
    assert np.allclose(KF.x, 0.0)

def test_predict():
    KF = SequentialKalmanFilter(1, 2)
    KF.x = np.array([[0.0], [2.0]])
    dt = 0.5

    KF.predict(dt)

    exp_x = np.array([[1.0], [2.0]])
    np.testing.assert_allclose(KF.x, exp_x)

# KF = SequentialKalmanFilter(1, 2)
# print(KF.x)
# KF.x = np.array([[0.0], [2.0]])
# print(KF.x)
# for i in range(5):
# KF.predict(0.5)
# print(KF.x)

def test_update():
    kf = SequentialKalmanFilter(dim_z=1, dim_x=2)
    R = np.array([[0.01]])
    
    huge_outlier = np.array([[100.0]])
    
    accepted = kf.update(huge_outlier, R)
    
    assert accepted is False