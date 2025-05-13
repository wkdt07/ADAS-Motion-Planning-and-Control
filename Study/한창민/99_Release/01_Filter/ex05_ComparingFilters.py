from ex01_AverageFilter import AverageFilter
from ex02_MovingAverageFilter import MovingAverageFilter
from ex03_LowPassFilter import LowPassFilter
from ex04_KalmanFilter import KalmanFilter
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.1, m=0.1, modelVariance=0.01, measureVariance=1.0, errorVariance_init=10.0):
        # 시스템 모델 계수
        self.A = 1.0
        self.B = step_time / m  # 제어 입력 계수
        self.C = 1.0
        self.D = 0.0

        # 노이즈 분산 설정
        self.Q = modelVariance + 0.01   # 모델 예측 잡음
        self.R = measureVariance -0.02 # 측정 잡음

        # 초기 상태값
        self.x_estimate = y_Measure_init  # 추정값 초기화
        self.P_estimate = errorVariance_init   # 추정 오차 공분산 초기화

    def prediction(self, u):
        # 예측 단계
        self.x_predict = self.A * self.x_estimate + self.B * u
        self.P_predict = self.A * self.P_estimate * self.A + self.Q

    def correction(self, y_measure):
        # 칼만 이득 계산
        K = self.P_predict * self.C / (self.C * self.P_predict * self.C + self.R)
        # 측정 기반 상태 보정
        self.x_estimate = self.x_predict + K * (y_measure - self.C * self.x_predict)
        # 추정 오차 공분산 갱신
        self.P_estimate = (1 - K * self.C) * self.P_predict

    def estimate(self, y_measure, input_u):
        # 전체 필터 수행
        self.prediction(input_u)
        self.correction(y_measure)
class LowPassFilter:
    def __init__(self, y_initial_measure, alpha=0.9):
        self.y_estimate = y_initial_measure
        self.alpha = alpha
        
 
    def estimate(self, y_measure):
        self.y_estimate = self.alpha *self.y_estimate + (1- self.alpha)*y_measure

class MovingAverageFilter:
    def __init__(self, y_initial_measure, num_average=10.0):
        self.y_estimate = y_initial_measure
        self.num_average = int(num_average) #윈도우 사이즈
               
        self.buff = [y_initial_measure]
    
    def estimate(self, y_measure):
           
        self.buff.append(y_measure)
       
        if len(self.buff) > self.num_average:
            self.buff.pop(0)
        self.y_estimate = sum(self.buff)/len(self.buff) # 지금까지의 평균
    
class AverageFilter:
    def __init__(self, y_initial_measure):
        self.sum = y_initial_measure  # 누적 합
        self.count = 1                # 측정 횟수
        self.y_estimate = y_initial_measure

    def estimate(self, y_measure):
        self.count += 1
        self.sum += y_measure
        self.y_estimate = self.sum / self.count  # 지금까지의 평균


if __name__ == "__main__":
    # 데이터 불러오기
    signal = pd.read_csv("01_filter/Data/example_KalmanFilter_1.csv")

    # 필터 인스턴스 초기화
    y_estimate_AF  = AverageFilter(signal.y_measure[0])
    y_estimate_MAF = MovingAverageFilter(signal.y_measure[0], num_average=10)
    y_estimate_LPF = LowPassFilter(signal.y_measure[0], alpha=0.9)
    y_estimate_KF  = KalmanFilter(signal.y_measure[0], step_time=0.1, m=1.0, modelVariance=0.01, measureVariance=1.0)

    # 출력 저장용 리스트
    t = []
    y_AF = []
    y_MAF = []
    y_LPF = []
    y_KF = []

    # 필터 루프
    for i, row in signal.iterrows():
        t.append(signal.time[i])

        # Average Filter
        y_estimate_AF.estimate(signal.y_measure[i])
        y_AF.append(y_estimate_AF.y_estimate)

        # Moving Average Filter
        y_estimate_MAF.estimate(signal.y_measure[i])
        y_MAF.append(y_estimate_MAF.y_estimate)

        # Low Pass Filter
        y_estimate_LPF.estimate(signal.y_measure[i])
        y_LPF.append(y_estimate_LPF.y_estimate)

        # Kalman Filter
        y_estimate_KF.estimate(signal.y_measure[i], signal.u[i])
        y_KF.append(y_estimate_KF.x_estimate)

    # 결과 시각화
    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure")
    plt.plot(t, y_AF,  'm-', label="Average Filter")
    plt.plot(t, y_MAF, 'b-', label="Moving Average Filter")
    plt.plot(t, y_LPF, 'c-', label="Low Pass Filter")
    plt.plot(t, y_KF,  'r-', label="Kalman Filter")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()