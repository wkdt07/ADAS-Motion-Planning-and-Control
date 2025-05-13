import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class MovingAverageFilter:
    def __init__(self, y_initial_measure, num_average):
        self.n = num_average
        self.history = [y_initial_measure]
        self.y_estimate = y_initial_measure

    def estimate(self, y_measure):
        self.history.append(y_measure)
        if len(self.history) > self.n + 1:
            x_k_n = self.history[-(self.n + 1)]
        else:
            x_k_n = self.history[0]


        # 수식 적용
        self.y_estimate = self.y_estimate + (1 / self.n) * (y_measure - x_k_n)


if __name__ == "__main__":
    #signal = pd.read_csv("week_01_filter/Data/example_Filter_1.csv")      
    signal = pd.read_csv("D:/99_Release/01_Filter/Data/example_Filter_2.csv")

    # y_estimate 컬럼 미리 생성
    signal["y_estimate"] = 0.0

    # 필터 객체 생성
    y_filter = MovingAverageFilter(signal.y_measure[0], num_average=5)  # 여기서 필터 길이 조절 가능

    # 이동 평균 필터 적용
    for i in range(len(signal)):
        y_filter.estimate(signal.y_measure[i])
        signal.loc[i, "y_estimate"] = y_filter.y_estimate

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
