import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class AverageFilter:
    def __init__(self, y_initial_measure):
        self.sum = y_initial_measure  # 총합
        self.count = 1                # 값의 개수
        self.y_estimate = y_initial_measure

    def estimate(self, y_measure):
        self.sum += y_measure
        self.count += 1
        self.y_estimate = self.sum / self.count  # 누적 평균 계산

if __name__ == "__main__":
    signal = pd.read_csv("C:/Users/pc/Desktop/IVS/motion planning/99_Release/01_Filter/Data/example_KalmanFilter_1.csv")

    signal["y_estimate"] = 0.0

    y_estimator = AverageFilter(signal.y_measure[0])

    for i, row in signal.iterrows():
        y_estimator.estimate(signal.y_measure[i])
        signal.at[i, "y_estimate"] = y_estimator.y_estimate

    # 시각화
    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure")
    plt.plot(signal.time, signal.y_estimate, 'b-', label="Estimate (Average)")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.grid(True)
    plt.axis("equal")
    plt.show()
