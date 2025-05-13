import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, 
                 y_Measure_init, 
                 step_time=0.1, 
                 m=10.0, 
                 b=2.0, 
                 k=100.0, 
                 Q_x=0.01, 
                 Q_v=0.01, 
                 R=5.0, 
                 errorCovariance_init=10.0):
        
        self.dt = step_time
        self.m = m
        self.b = b
        self.k = k

        # 상태 공간 모델 정의
        self.A = np.array([[1.0, self.dt],
                           [-self.k/self.m * self.dt, 1.0 - self.b/self.m * self.dt]])
        
        self.B = np.array([[0.0], [self.dt/self.m]])
        self.C = np.array([[1.0, 0.0]])  # 측정은 위치만 측정된다고 가정
        self.Q = np.array([[Q_x, 0.0],
                           [0.0, Q_v]])
        self.R = R
        self.x_estimate = np.array([[y_Measure_init], [0.0]])  # [위치, 속도]
        self.P_estimate = np.eye(2) * errorCovariance_init

    def estimate(self, y_measure, input_u):
        # 예측 단계
        x_predict = self.A @ self.x_estimate + self.B * input_u
        P_predict = self.A @ self.P_estimate @ self.A.T + self.Q

        # 측정 업데이트 단계
        y_predict = self.C @ x_predict
        residual = y_measure - y_predict
        S = self.C @ P_predict @ self.C.T + self.R
        K = P_predict @ self.C.T @ np.linalg.inv(S)

        self.x_estimate = x_predict + K @ residual
        self.P_estimate = (np.eye(2) - K @ self.C) @ P_predict

if __name__ == "__main__":
    #원하는 튜닝 파라미터 설정
    Q_x = 0.1  # 위치 예측 노이즈
    Q_v = 10  # 속도 예측 노이즈
    R = 5     # 측정 노이즈

    signal = pd.read_csv("01_filter/Data/example08.csv")
    signal['y_estimate'] = 0.0

    y_estimate = KalmanFilter(signal.y_measure[0], Q_x=Q_x, Q_v=Q_v, R=R)

    for i, row in signal.iterrows():
        y_estimate.estimate(row.y_measure, row.u)
        signal.at[i, 'y_estimate'] = y_estimate.x_estimate[0][0]  # 위치 추정

    #그래프 출력
    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measured Position")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Estimated Position")
    plt.xlabel('time (s)')
    plt.ylabel('position (m)')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
