import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.01, m=0.1, modelVariance=0.01, measureVariance=1.0, errorVariance_init=10.0):
        # 시스템 모델 계수
        self.A = 1.0
        self.B = step_time / m  # 제어 입력 계수
        self.C = 1.0
        self.D = 0.0

        # 노이즈 분산 설정
        self.Q = modelVariance + 0.01   # 모델 예측 잡음
        self.R = measureVariance -0.02 # 측정 잡음

        # 초기 상태값
        self.x_estimate = y_Measure_init  # 추정값 초기화
        self.P_estimate = errorVariance_init   # 추정 오차 공분산 초기화

    def prediction(self, u):
        # 예측 단계
        self.x_predict = self.A * self.x_estimate + self.B * u
        self.P_predict = self.A * self.P_estimate * self.A + self.Q

    def correction(self, y_measure):
        # 칼만 이득 계산
        K = self.P_predict * self.C / (self.C * self.P_predict * self.C + self.R)
        # 측정 기반 상태 보정
        self.x_estimate = self.x_predict + K * (y_measure - self.C * self.x_predict)
        # 추정 오차 공분산 갱신
        self.P_estimate = (1 - K * self.C) * self.P_predict

    def estimate(self, y_measure, input_u):
        # 전체 필터 수행
        self.prediction(input_u)
        self.correction(y_measure)

if __name__ == "__main__":
    # CSV 파일 읽기
    signal = pd.read_csv("01_filter/Data/example_KalmanFilter_1.csv")
    signal["y_estimate"] = 0.0  # 결과 저장용 열 추가

    # 칼만 필터 초기화 (파라미터 조절 가능)
    kf = KalmanFilter(
        y_Measure_init=signal.y_measure[0],
        step_time=0.1,
        m=1.0,
        modelVariance=0.01,
        measureVariance=1.0,
        errorVariance_init=10.0
    )

    # 필터 수행 루프
    for i, row in signal.iterrows():
        kf.estimate(signal.y_measure[i], signal.u[i])
        signal.loc[i, "y_estimate"] = kf.x_estimate

    # 결과 시각화
    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
