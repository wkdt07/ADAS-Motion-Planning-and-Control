from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=0.4, D_Gain=0.9, I_Gain=0.02):
        self.Kp = P_Gain
        self.Kd = D_Gain
        self.Ki = I_Gain
        self.dt = step_time

        self.prev_error = reference - measure
        self.integral_error = 0.0
        self.u = 0.0

    def ControllerInput(self, reference, measure):
        error = reference - measure
        self.integral_error += error * self.dt
        d_error = (error - self.prev_error) / self.dt

        self.u = self.Kp * error + self.Ki * self.integral_error + self.Kd * d_error
        self.prev_error = error

        
class KalmanFilter:
    def __init__(self, initial_y, A=1, B=0.1, H=1, Q=0.01, R=0.1):
        # 초기 상태 (위치, 속도 추정 가능하도록 확장 가능)
        self.x_estimate = np.array([[initial_y]])  # 상태 추정값 (y 위치만 추정)
        self.P = np.array([[1]])  # 공분산 초기값

        # 시스템 행렬
        self.A = np.array([[A]])  # 상태 전이 행렬
        self.B = np.array([[B]])  # 제어 입력 행렬
        self.H = np.array([[H]])  # 측정 행렬

        # 잡음 공분산
        self.Q = np.array([[Q]])  # 프로세스 노이즈
        self.R = np.array([[R]])  # 측정 노이즈

    def estimate(self, measurement, u):
        # 1. Prediction
        self.x_estimate = self.A @ self.x_estimate + self.B * u
        self.P = self.A @ self.P @ self.A.T + self.Q

        # 2. Update
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        y = measurement - (self.H @ self.x_estimate)
        self.x_estimate = self.x_estimate + K @ y
        self.P = (np.eye(self.H.shape[1]) - K @ self.H) @ self.P

        
        
if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    estimated_y = []
    time = []
    step_time = 0.1
    simulation_time = 30   
    plant = VehicleModel(step_time, 0.25, 0.99, 0.05)
    estimator = KalmanFilter(plant.y_measure[0][0])
    controller = PID_Controller(target_y, plant.y_measure[0][0], step_time)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        estimated_y.append(estimator.x_estimate[0][0])
        estimator.estimate(plant.y_measure[0][0],controller.u)
        controller.ControllerInput(target_y, estimator.x_estimate[0][0])
        plant.ControlInput(controller.u)
    
    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r:',label = "Vehicle Position(Measure)")
    plt.plot(time, estimated_y,'c-',label = "Vehicle Position(Estimator)")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
