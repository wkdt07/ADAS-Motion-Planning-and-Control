from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=10.0, D_Gain=50.0, I_Gain=0.02):
        # Code
        self.Kp = P_Gain
        self.Ki = I_Gain
        self.Kd = D_Gain

        self.target_y = reference
        self.y_measure = measure
        self.dt = step_time

        self.error = reference - measure
        self.i_err = 0

    def ControllerInput(self, reference, measure):
        # Code
        self.target_y = reference
        self.y_measure = measure
        self.error_gap = (reference - measure) - self.error
        self.error = reference - measure

        self.i_err += self.error

        self.up = self.Kp * self.error
        self.ui = self.Ki * self.i_err
        self.ud = self.Kd * self.error_gap

        self.u = self.up + self.ui + self.ud
        
class KalmanFilter:
    def __init__(self, initial_measurement, dt=0.1):
        # 상태: 위치(y), 속도(v)
        self.x_estimate = np.array([[initial_measurement], [0.0]])  # 초기 추정값
        self.dt = dt

        # 시스템 행렬
        self.A = np.array([[1.0, dt],
                           [0.0, 1.0]])  # 상태 전이 행렬
        self.B = np.array([[0.0],
                           [dt]])        # 입력 행렬
        self.C = np.array([[1.0, 0.0]])  # 측정 행렬

        self.P = np.eye(2)              # 초기 공분산
        self.Q = np.array([[1e-4, 0.0],
                           [0.0, 1e-2]])  # 시스템 노이즈
        self.R = np.array([[0.1]])       # 측정 노이즈

    def estimate(self, measurement, control_input):
        # 예측 단계
        self.x_estimate = self.A @ self.x_estimate + self.B * control_input
        self.P = self.A @ self.P @ self.A.T + self.Q

        # 측정 업데이트 단계
        y_tilde = measurement - self.C @ self.x_estimate
        S = self.C @ self.P @ self.C.T + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)

        self.x_estimate = self.x_estimate + K @ y_tilde
        self.P = (np.eye(2) - K @ self.C) @ self.P

if __name__ == "__main__":
    target_y = 0.0
    measure_y = []
    estimated_y = []
    time = []
    step_time = 0.1
    simulation_time = 30

    plant = VehicleModel(step_time, 0.25, 0.99, 0.05)
    estimator = KalmanFilter(plant.y_measure[0][0], dt=step_time)
    controller = PID_Controller(target_y, plant.y_measure[0][0], step_time)

    for i in range(int(simulation_time / step_time)):
        time.append(step_time * i)
        measure_y.append(plant.y_measure[0][0])
        estimated_y.append(estimator.x_estimate[0][0])

        estimator.estimate(plant.y_measure[0][0], controller.u if hasattr(controller, 'u') else 0.0)
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
