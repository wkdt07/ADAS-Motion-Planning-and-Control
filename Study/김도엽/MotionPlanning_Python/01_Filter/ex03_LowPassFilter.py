import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class LowPassFilter:
    def __init__(self, y_initial_measure, alpha=0.9):
        self.alpha = alpha
        self.y_estimate = y_initial_measure  # 초기값으로 추정값 설정

    def estimate(self, y_measure):
        self.y_estimate = self.alpha * self.y_estimate + (1 - self.alpha) * y_measure

if __name__ == "__main__":     
    # CSV 파일 읽기
    signal = pd.read_csv("C:/Users/pc/Desktop/IVS/motion planning/99_Release/01_Filter/Data/example_KalmanFilter_1.csv")

    # y_estimate 컬럼 초기화
    signal["y_estimate"] = 0.0

    # 필터 객체 생성
    y_estimate = LowPassFilter(signal.y_measure[0], alpha=0.8)

    # 필터 적용
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i])
        signal.at[i, "y_estimate"] = y_estimate.y_estimate

    # 시각화
    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Estimate (Low Pass)")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
