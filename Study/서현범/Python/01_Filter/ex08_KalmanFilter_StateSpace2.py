import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self,
                    y_Measure_init,
                    step_time=0.1,
                    m=10.0,
                    k=100.0,
                    b=2.0,
                    Q_x=0.02,
                    Q_v=0.05,
                    R=5.0,
                    errorCovariance_init=10.0):
        # Code 
        A_cont = np.array([[0.0,        1.0],
                           [-k/m,   -b/m]])
        B_cont = np.array([[0.0],
                           [1.0/m]])   

        self.A = np.eye(2) + step_time * A_cont
        self.B = step_time * B_cont
        
        self.C = np.array([[1.0, 0.0]])

        self.Q = np.diag([Q_x, Q_v])
        self.R = R

        # 초기값
        self.x_estimate = np.array([[y_Measure_init],[0.0]])
        self.P_estimate = np.eye(2) * errorCovariance_init

    def estimate(self, y_measure, input_u):
        # Code

        # Prediction
        x_pred = self.A @ self.x_estimate + self.B * input_u
        P_pred = self.A @ self.P_estimate @ self.A.T + self.Q

        # Update
        S = self.C @ P_pred @ self.C.T + self.R
        K = P_pred @ self.C.T @ np.linalg.inv(S)

        self.x_estimate = x_pred + K @ (y_measure - self.C @ x_pred)
        self.P_estimate = (np.eye(2) - K @ self.C) @ P_pred
        # self.P_estimate = (1 - K @ self.C) @ P_pred # 이거는 1차원일때만 가능
        
if __name__ == "__main__":
    signal = pd.read_csv("./Data/example08.csv")

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



