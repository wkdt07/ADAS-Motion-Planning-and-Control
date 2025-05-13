import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class LowPassFilter:
    def __init__(self, y_initial_measure, alpha):
        self.y_estimate = y_initial_measure
        self.alpha = alpha
 
    def estimate(self, y_measure):
        self.y_estimate = self.alpha * self.y_estimate + (1 - self.alpha) * y_measure


if __name__ == "__main__":
    #signal = pd.read_csv("01_filter/Data/example_Filter_1.csv")
    #signal = pd.read_csv("01_filter/Data/example_Filter_2.csv")     
    
    signal = pd.read_csv("D:/99_Release/01_Filter/Data/example_Filter_3.csv")

    signal["y_estimate"] = 0.0  # 추정값 컬럼 초기화
    y_estimate = LowPassFilter(signal.y_measure[0], alpha=0.7)

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
