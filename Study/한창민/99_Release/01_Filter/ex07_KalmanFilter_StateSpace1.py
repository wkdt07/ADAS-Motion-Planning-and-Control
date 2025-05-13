import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.01, m=1.0, Q_x=0.02, Q_v=0.05, R=5.0, errorCovariance_init=10.0):
        self.A = np.array([[1.0, step_time], 
                           [0.0, 1.0]])             # 상태전이 행렬
        self.B = np.array([[0.0], 
                           [step_time / m]])        # 입력 행렬
        self.C = np.array([[1.0, 0.0]])             # 측정 행렬 (위치만 측정)
        self.Q = np.array([[Q_x, 0.0], 
                           [0.0, Q_v]])             # 프로세스 노이즈 공분산
        self.R = np.array([[R]])                    # 측정 노이즈 공분산

        self.x_estimate = np.array([[y_Measure_init], [0.0]])  # 초기 추정 상태 [x, v]
        self.P_estimate = np.eye(2) * errorCovariance_init     # 초기 오차 공분산

    def estimate(self, y_measure, input_u):
        # ===== 1. Prediction =====
        u = np.array([[input_u]])
        x_predict = self.A @ self.x_estimate + self.B @ u
        P_predict = self.A @ self.P_estimate @ self.A.T + self.Q

        # ===== 2. Update (Correction) =====
        K = P_predict @ self.C.T @ np.linalg.inv(self.C @ P_predict @ self.C.T + self.R)
        y = np.array([[y_measure]]) - (self.C @ x_predict)
        self.x_estimate = x_predict + K @ y
        self.P_estimate = (np.eye(2) - K @ self.C) @ P_predict

if __name__ == "__main__":
    signal = pd.read_csv("01_filter/Data/example07.csv")
    signal["y_estimate"] = 0.0  # 결과 저장 열 추가

    y_estimate = KalmanFilter(signal.y_measure[0])

    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i], signal.u[i])
        signal.loc[i, "y_estimate"] = y_estimate.x_estimate[0][0]  # 위치 추정값만 저장

    # 시각화
    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure (Position)")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Estimate (Kalman x̂)")
    plt.xlabel('time (s)')
    plt.ylabel('position')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.title("Kalman Filter (Position Estimate)")
    plt.show()
