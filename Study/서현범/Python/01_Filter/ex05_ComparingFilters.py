from ex01_AverageFilter import AverageFilter
from ex02_MovingAverageFilter import MovingAverageFilter
from ex03_LowPassFilter import LowPassFilter
from ex04_KalmanFilter import KalmanFilter
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
class AverageFilter:
    def __init__(self, y_initial_measure):
        self.y_estimate = y_initial_measure
        # Code
        self.k = 1

                  
    def estimate(self, y_measure):
        # Code
        self.k += 1
        k = self.k
        self.y_estimate = (self.y_estimate*(k-1)/k) + (y_measure/k)

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
        
class LowPassFilter:
    def __init__(self, y_initial_measure, alpha=0.9):
        self.y_estimate = y_initial_measure
        self.aplha = alpha
        # Code
 
    def estimate(self, y_measure):
        # Code
        self.y_estimate = self.aplha * self.y_estimate + (1 - self.aplha) * y_measure

    
class KalmanFilter:
    def __init__(self, y_Measure_init, step_time = 0.1, m = 0.1, modelVariance = 0.01, measureVariance = 1.0, errorVariance_init = 10.0):
        self.A = (1.0)
        self.B = step_time/m
        self.C = 1.0
        self.D = 0.0
        self.Q = modelVariance
        self.R = measureVariance
        self.x_estimate = y_Measure_init
        self.P_estimate = errorVariance_init


    '''
    delta_v/delta_t = a = u/m
    만약 v를 x라 한다면

    x_k = x_(k-1) + delta_t/m * u_k
    x_k = A * x_(k-1) + B * u_k
    '''
    def estimate(self, y_measure, input_u):
        # Prediction
        x_pred = self.A * self.x_estimate + self.B * input_u
        P_pred = self.A * self.P_estimate * self.A + self.Q
        # Update
        K = P_pred * self.C / (self.C * P_pred * self.C + self.R)
        self.x_estimate = x_pred + K * (y_measure - self.C * x_pred)
        self.P_estimate = (1 - K * self.C) * P_pred
        

if __name__ == "__main__":
    t = []
    y_AF = []
    y_MAF = []
    y_LPF = []
    y_KF = []
    
    signal = pd.read_csv("./Data/example_KalmanFilter_1.csv")

    #Code
    y_estimate_AF = AverageFilter(signal.y_measure[0])
    y_estimate_MAF = MovingAverageFilter(signal.y_measure[0])
    y_estimate_LPF = LowPassFilter(signal.y_measure[0],0.3) 
    y_estimate_KF = KalmanFilter(signal.y_measure[0])
    
    for i, row in signal.iterrows():
        t.append(signal.time[i])
        # Averageg filter
        y_estimate_AF.estimate(signal.y_measure[i])
        y_AF.append(y_estimate_AF.y_estimate)
        # Moving Average filter
        y_estimate_MAF.estimate(signal.y_measure[i])
        y_MAF.append(y_estimate_MAF.y_estimate)
        # Low pass filter
        y_estimate_LPF.estimate(signal.y_measure[i])
        y_LPF.append(y_estimate_LPF.y_estimate)
        # Kalman filter
        y_estimate_KF.estimate(signal.y_measure[i],signal.u[i])
        y_KF.append(y_estimate_KF.x_estimate)
        

    plt.figure()
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(t, y_AF,'m-',label = "Average Filter")
    plt.plot(t, y_MAF,'b-',label = "Moving Average Filter")
    plt.plot(t, y_LPF,'c-',label = "Low Pass Filter")
    plt.plot(t, y_KF,'r-',label = "Kalman Filter")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



