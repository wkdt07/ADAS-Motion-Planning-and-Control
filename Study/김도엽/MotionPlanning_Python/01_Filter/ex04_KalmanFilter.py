import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.01, m=1.0, modelVariance=0.01, measureVariance=0.9, errorVariance_init=10.0):
        self.A = 1.0
        self.B = step_time / m
        self.C = 1.0
        self.Q = modelVariance     # Process noise variance
        self.R = measureVariance   # Measurement noise variance
        self.x_estimate = y_Measure_init  # 초기 속도 추정
        self.P_estimate = errorVariance_init  # 초기 추정 오차 공분산

    def estimate(self, y_measure, input_u):
        # 1. Prediction
        x_predict = self.A * self.x_estimate + self.B * input_u
        P_predict = self.A * self.P_estimate * self.A + self.Q

        # 2. Kalman Gain
        K = P_predict * self.C / (self.C * P_predict * self.C + self.R)

        # 3. Update
        self.x_estimate = x_predict + K * (y_measure - self.C * x_predict)
        self.P_estimate = (1 - K * self.C) * P_predict


if __name__ == "__main__":
    # CSV 파일 로딩
    signal = pd.read_csv("C:/Users/pc/Desktop/IVS/motion planning/99_Release/01_Filter/Data/example_KalmanFilter_1.csv")

    # 추정값 컬럼 초기화
    signal["y_estimate"] = 0.0

    # Kalman 필터 객체 생성 (초기값은 첫 측정값)
    kf = KalmanFilter(signal.y_measure[0])

    # 칼만 필터 적용
    for i, row in signal.iterrows():
        kf.estimate(signal.y_measure[i], signal.u[i])
        signal.at[i, "y_estimate"] = kf.x_estimate

    # 결과 시각화
    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Estimate (Kalman)")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
