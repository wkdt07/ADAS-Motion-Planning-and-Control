import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.1, m=0.1, modelVariance=0.01, measureVariance=1.0, errorVariance_init=10.0):
        self.dt = step_time
        self.m = m
        self.A = 1.0  # 상태 전이 행렬 (속도 → 속도)
        self.B = self.dt / self.m  # 입력 행렬 (힘 → 속도 변화)
        self.C = 1.0  # 측정 행렬
        self.Q = modelVariance  # 프로세스 노이즈 분산
        self.R = measureVariance  # 측정 노이즈 분산
        self.x_estimate = y_Measure_init  # 초기 속도 추정
        self.P_estimate = errorVariance_init  # 초기 오차 공분산

    def estimate(self, y_measure, input_u):
        # === Prediction ===
        x_predict = self.A * self.x_estimate + self.B * input_u
        P_predict = self.A * self.P_estimate * self.A + self.Q

        # === Update ===
        K = P_predict * self.C / (self.C * P_predict * self.C + self.R)  # 칼만 이득
        self.x_estimate = x_predict + K * (y_measure - self.C * x_predict)
        self.P_estimate = (1 - K * self.C) * P_predict


if __name__ == "__main__":
    # CSV 데이터 로드
    signal = pd.read_csv("C:/Users/pc/Desktop/IVS/motion planning/99_Release/01_Filter/Data/example0.csv")

    # y_estimate 컬럼 초기화
    signal["y_estimate"] = 0.0

    # === 칼만 필터 파라미터 튜닝 ===
    step_time = 0.1          # 샘플링 주기 (초)
    m = 0.1                  # 질량 (kg)
    Q = 0.01                 # 모델 노이즈 분산 (작을수록 모델을 더 신뢰함)
    R = 5.0                  # 측정 노이즈 분산 (클수록 측정을 덜 신뢰함)
    P_init = 10.0            # 초기 오차 공분산

    # 칼만 필터 객체 초기화
    kf = KalmanFilter(signal.y_measure[0], step_time, m, Q, R, P_init)

    # 추정 반복
    for i, row in signal.iterrows():
        kf.estimate(signal.y_measure[i], signal.u[i])
        signal.at[i, "y_estimate"] = kf.x_estimate

    # 결과 시각화
    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure (noisy)")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Kalman Estimate")
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.title("Kalman Filter for Mass-Force System")
    plt.show()
