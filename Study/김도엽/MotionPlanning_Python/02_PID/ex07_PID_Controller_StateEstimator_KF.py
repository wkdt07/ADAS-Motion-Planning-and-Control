from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=0.4, D_Gain=0.9, I_Gain=0.02):
        self.Kp = P_Gain
        self.Kd = D_Gain
        self.Ki = I_Gain
        self.step_time = step_time
        self.prev_error = reference - measure
        self.integral_error = 0.0
        self.u = 0.0

    def ControllerInput(self, reference, measure):
        error = reference - measure
        self.integral_error += error * self.step_time
        d_error = (error - self.prev_error) / self.step_time
        self.u = self.Kp * error + self.Ki * self.integral_error + self.Kd * d_error
        self.prev_error = error

class KalmanFilter:
    def __init__(self, init_y):
        self.dt = 0.1
        self.A = np.array([[1]])               # 상태 전이 행렬
        self.B = np.array([[self.dt]])         # 입력 행렬
        self.H = np.array([[1]])               # 관측 행렬
        self.Q = np.array([[1e-4]])            # 프로세스 노이즈 공분산
        self.R = np.array([[0.01]])            # 측정 노이즈 공분산
        self.P = np.array([[1]])               # 오차 공분산
        self.x_estimate = np.array([[init_y]]) # 초기 추정값

    def estimate(self, z, u):
        # Prediction
        x_pred = self.A @ self.x_estimate + self.B * u
        P_pred = self.A @ self.P @ self.A.T + self.Q

        # Update
        K = P_pred @ self.H.T @ np.linalg.inv(self.H @ P_pred @ self.H.T + self.R)
        self.x_estimate = x_pred + K @ (np.array([[z]]) - self.H @ x_pred)
        self.P = (np.eye(1) - K @ self.H) @ P_pred

if __name__ == "__main__":
    target_y = 0.0
    measure_y = []
    estimated_y = []
    time = []
    step_time = 0.1
    simulation_time = 30   

    plant = VehicleModel(step_time, 0.25, 0.99, 0.05)
    estimator = KalmanFilter(plant.y_measure[0][0])
    controller = PID_Controller(target_y, plant.y_measure[0][0], step_time)
    
    for i in range(int(simulation_time / step_time)):
        t = step_time * i
        time.append(t)

        y_meas = plant.y_measure[0][0]
        measure_y.append(y_meas)

        estimator.estimate(y_meas, controller.u)
        estimated_y.append(estimator.x_estimate[0][0])

        controller.ControllerInput(target_y, estimator.x_estimate[0][0])
        plant.ControlInput(controller.u)

    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="Reference")
    plt.plot(time, measure_y, 'r:', label="Measurement (Sensor)")
    plt.plot(time, estimated_y, 'c-', label="Kalman Estimate")
    plt.xlabel('Time (s)')
    plt.ylabel('Y Position')
    plt.title("PID with Kalman Filter State Estimation")
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()