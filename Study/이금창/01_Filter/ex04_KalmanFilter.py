import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.01, m=0.1, modelVariance=0.01, measureVariance=1.0, errorVariance_init=10.0):
        self.A = 1.0 + step_time
        self.B = step_time / m
        self.C = 1.0
        self.D = 0.0
        self.Q = modelVariance  # 모델 노이즈 공분산
        self.R = measureVariance  # 측정 노이즈 공분산
        self.x_estimate = y_Measure_init  # 초기 추정값
        self.P_estimate = errorVariance_init  # 초기 추정 공분산

    def estimate(self, y_measure, input_u):
        # Prediction (예측)
        x_predict = self.A * self.x_estimate + self.B * input_u
        P_predict = self.A * self.P_estimate * self.A + self.Q

        # Kalman Gain
        K = P_predict * self.C / (self.C * P_predict * self.C + self.R)

        # Update (보정)
        self.x_estimate = x_predict + K * (y_measure - self.C * x_predict)
        self.P_estimate = (1 - K * self.C) * P_predict


        

if __name__ == "__main__":
    signal = pd.read_csv("D:/99_Release/01_Filter/Data/example_KalmanFilter_1.csv")  # 파일명 확인 주의
    signal["y_estimate"] = 0.0  # 결과 저장할 열

    kf = KalmanFilter(signal.y_measure[0])
    for i, row in signal.iterrows():
        kf.estimate(signal.y_measure[i], signal.u[i])
        signal.y_estimate[i] = kf.x_estimate

    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('velocity (m/s)')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



