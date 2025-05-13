import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.1, m=1.0, Q_x=0.02, Q_v=0.05, R=5.0, errorCovariance_init=10.0):
        T = step_time
        self.A = np.array([[1.0, T], [0.0, 1.0]])
        self.B = np.array([[0.5 * T**2 / m], [T / m]])
        self.C = np.array([[1.0, 0.0]])  # 위치만 측정
        self.Q = np.array([[Q_x, 0.0], [0.0, Q_v]])
        self.R = R
        self.x_estimate = np.array([[y_Measure_init], [0.0]])  # 초기 위치, 초기 속도 0
        self.P_estimate = np.eye(2) * errorCovariance_init

    def estimate(self, y_measure, input_u):
        # Prediction
        x_predict = self.A @ self.x_estimate + self.B * input_u
        P_predict = self.A @ self.P_estimate @ self.A.T + self.Q

        # Kalman Gain
        S = self.C @ P_predict @ self.C.T + self.R
        K = P_predict @ self.C.T @ np.linalg.inv(S)

        # Update
        y_predict = self.C @ x_predict
        self.x_estimate = x_predict + K @ (y_measure - y_predict)
        self.P_estimate = (np.eye(2) - K @ self.C) @ P_predict

if __name__ == "__main__":
    signal = pd.read_csv("C:/Users/pc/Desktop/IVS/motion planning/99_Release/01_Filter/Data/example07.csv")

    signal["y_estimate"] = 0.0
    kf = KalmanFilter(signal.y_measure[0])

    for i, row in signal.iterrows():
        kf.estimate(signal.y_measure[i], signal.u[i])
        signal.at[i, "y_estimate"] = kf.x_estimate[0, 0]  # 위치 추정값 저장

    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measured Position")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Estimated Position")
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend(loc="best")
    plt.grid(True)
    plt.title("Kalman Filter - Position Estimation")
    plt.show()
