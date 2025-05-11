import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 이동 평균 필터
# y_measure: 실제값 xk
# y_estimate: 추정값 xk_hat
# num_average : 이동 슬라이드 크기 (n)
# y_estimate = y_estimate + (y_measure - y_measure_예전거)/num_average
class MovingAverageFilter:
    def __init__(self, y_initial_measure, num_average=2.0):
        self.y_estimate = y_initial_measure
        # Code
        self.num_average = num_average
        self.y_measure_old = y_initial_measure
        self.buffer = [y_initial_measure]
    def estimate(self, y_measure):
        # Code
        if len(self.buffer) < self.num_average: # 아직 버퍼 다 안 참
            self.buffer.append(y_measure)
            # self.y_estimate = self.y_estimate + (y_measure - self.buffer[0]) / len(self.buffer)
            oldest = self.buffer[0]  # 먼저 꺼내두고
            self.buffer.append(y_measure)
            self.y_estimate += (y_measure - oldest) / len(self.buffer)
        else:
            old_value = self.buffer.pop(0)
            self.buffer.append(y_measure)
            self.y_estimate = self.y_estimate + (y_measure - old_value) / self.num_average
        
    
if __name__ == "__main__":
    #signal = pd.read_csv("week_01_filter/Data/example_Filter_1.csv")      
    signal = pd.read_csv("./Data/example_Filter_2.csv")

    y_estimate = MovingAverageFilter(signal.y_measure[0])
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i])
        signal.y_estimate[i] = y_estimate.y_estimate

    plt.figure()
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



