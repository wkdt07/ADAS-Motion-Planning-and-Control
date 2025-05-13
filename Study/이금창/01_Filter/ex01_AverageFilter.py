import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class AverageFilter:
    def __init__(self, y_initial_measure):
        self.k = 1
        self.y_estimate = y_initial_measure

    def estimate(self, y_measure):
        self.k += 1
        self.y_estimate = ((self.k - 1) / self.k) * self.y_estimate + (1 / self.k) * y_measure



if __name__ == "__main__":
    signal = pd.read_csv("D:/99_Release/01_Filter/Data/example_Filter_1.csv")
    # signal = pd.read_csv("01_filter/Data/example_Filter_2.csv")

    # 예측값 컬럼 추가 (초기화)
    signal["y_estimate"] = 0.0

    # 필터 객체 생성
    y_filter = AverageFilter(signal.y_measure[0])

    for i in range(len(signal)):
        y_filter.estimate(signal.y_measure[i])
        signal.loc[i, "y_estimate"] = y_filter.y_estimate

    # 시각화
    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Estimate")
    plt.ylim(-5, 15)
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.grid(True)
    plt.show()
