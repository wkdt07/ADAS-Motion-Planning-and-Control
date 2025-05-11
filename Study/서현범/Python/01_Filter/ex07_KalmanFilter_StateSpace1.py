import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

'''
이번엔 v 기준이 아닌 x 기준으로 바꾸기
X = np.array([[x], [v]]) # (x,v)행렬(열행렬)
x_(k+1) = A * x_k + B * u_k 
x_(k+1) = np.array([[1.0, delta_t], [0.0, 1.0]]) * x_k + np.array([[0.0], [delta_t/m]]) * u_k
위에가 상태 방정식

<측정방정식>
y_k = C * x_k + D * v_k
y_k = np.array([[1.0, 0.0]]) * x_k + 0.0 * v_k

@ -> 파이썬 행렬 곱
'''
class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.1, m=1.0, Q_x=0.02, Q_v=0.05, R=5.0, errorCovariance_init=10.0):
        self.A = np.array([[1.0, step_time], [0.0, 1.0]])
        self.B = np.array([[0.0], [step_time/m]])
        self.C = np.array([[1.0, 0.0]])
        self.D = 0.0
        self.Q = np.array([[Q_x, 0.0], [0.0, Q_v]])
        self.R = R
        self.x_estimate = np.array([[y_Measure_init],[0.0]])
        self.P_estimate = np.array([[errorCovariance_init, 0.0],[0.0, errorCovariance_init]])

    def estimate(self, y_measure, input_u):
        # Prediction
        x_pred = self.A @ self.x_estimate + self.B * input_u
        P_pred = self.A @ self.P_estimate @ self.A.T + self.Q

        # Update
        K = P_pred @ self.C.T @ np.linalg.inv(self.C @ P_pred @ self.C.T + self.R)
        self.x_estimate = x_pred + K @ (y_measure - self.C @ x_pred)
        self.P_estimate = (np.eye(2) - K @ self.C) @ P_pred
        # self.P_estimate = (1 - K @ self.C) @ P_pred # 이거는 1차원일때만 가능



if __name__ == "__main__":
    signal = pd.read_csv("./Data/example07.csv")

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



