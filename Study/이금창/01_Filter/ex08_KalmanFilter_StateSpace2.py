import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, init_x):
        self.dt = 0.1
        self.A = np.array([[1, self.dt],
                           [-100/10*self.dt, 1 - 2/10*self.dt]])
        self.B = np.array([[0],
                           [self.dt/10]])
        self.H = np.array([[1, 0]])
        self.x_estimate = np.array([[init_x],
                                    [0]])  # 초기 속도 0
        self.P = np.eye(2)
        self.Q = np.eye(2) * 0.001  # 시스템 잡음
        self.R = np.array([[0.1]])  # 측정 잡음

    def estimate(self, z, u):
        # 예측
        self.x_estimate = np.dot(self.A, self.x_estimate) + self.B * u
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

        # 갱신
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        y = z - np.dot(self.H, self.x_estimate)
        self.x_estimate += np.dot(K, y)
        self.P = np.dot((np.eye(2) - np.dot(K, self.H)), self.P)

        
if __name__ == "__main__":
    signal = pd.read_csv("01_filter/Data/example08.csv")

    y_estimate = KalmanFilter(signal.y_measure[0])
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i],signal.u[i])
        signal.y_estimate[i] = y_estimate.x_estimate[0][0]

    plt.figure()
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



