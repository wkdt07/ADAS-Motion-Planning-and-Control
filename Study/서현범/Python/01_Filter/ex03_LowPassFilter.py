import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class LowPassFilter:
    def __init__(self, y_initial_measure, alpha=0.9):
        self.y_estimate = y_initial_measure
        self.aplha = alpha
        # Code
 
    def estimate(self, y_measure):
        # Code
        self.y_estimate = self.aplha * self.y_estimate + (1 - self.aplha) * y_measure


if __name__ == "__main__":
    #signal = pd.read_csv("01_filter/Data/example_Filter_1.csv")
    #signal = pd.read_csv("01_filter/Data/example_Filter_2.csv")      
    signal = pd.read_csv("./Data/example_Filter_3.csv")

    y_estimate = LowPassFilter(signal.y_measure[0],0.3) # 알파값 여기서 설정
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



