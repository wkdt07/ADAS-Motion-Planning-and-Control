import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class MovingAverageFilter:
    def __init__(self, y_initial_measure, num_average=2.0):
        self.num_average = int(num_average)                # 평균 낼 개수
        self.buffer = [y_initial_measure]                  # 초기 버퍼에 첫 값 저장
        self.y_estimate = y_initial_measure                # 초기 추정값

    def estimate(self, y_measure):
        self.buffer.append(y_measure)                      # 새 값 추가
        if len(self.buffer) > self.num_average:
            self.buffer.pop(0)                             # 가장 오래된 값 제거
        self.y_estimate = np.mean(self.buffer)             # 이동 평균 계산

if __name__ == "__main__":
    # CSV 파일 로드
    signal = pd.read_csv("C:/Users/pc/Desktop/IVS/motion planning/99_Release/01_Filter/Data/example_KalmanFilter_1.csv")

    # y_estimate 컬럼이 없을 수 있으므로 초기화
    signal["y_estimate"] = 0.0


    y_estimate = MovingAverageFilter(signal.y_measure[0], num_average=2.0)

    # 이동 평균 계산
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i])
        signal.at[i, "y_estimate"] = y_estimate.y_estimate  # 안전한 방식

    # 시각화
    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Estimate (Moving Avg)")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()




