from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=0.4, D_Gain=0.9, I_Gain=0.1):
        # Code
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.I_Gain = I_Gain
        self.step_time = step_time
        self.previous_error = 0.0
        self.u = 0.0
        self.integral = 0.0
        self.e_prev = 0.0
 
        self.integral = 0.0
    
    def ControllerInput(self, reference, measure):
        # Code
        self.reference = reference
        self.measure = measure
        e = reference - measure
        self.u = self.P_Gain * e + self.D_Gain * (e - self.e_prev) / self.step_time + self.I_Gain * self.integral
        self.e_prev = e # 이전 에러를 저장해둠
        self.integral += e * self.step_time # 적분값을 업데이트
        
class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.1, m=1.0, Q_x=0.02, Q_v=0.05, R=5.0, errorCovariance_init=10.0):
        self.A = np.array([[1.0, step_time], [0.0, 1.0]])
        self.B = np.array([[0.0], [step_time/m]])
        self.C = np.array([[1.0, 0.0]])
        self.D = 0.0
        self.Q = np.array([[Q_x, 0.0], [0.0, Q_v]])
        self.R = R
        self.x_estimate = np.array([[y_Measure_init],[0.0]])
        self.P_estimate = np.array([[errorCovariance_init, 0.0],[0.0, errorCovariance_init]])

    def estimate(self, y_measure, input_u):
        # Prediction
        x_pred = self.A @ self.x_estimate + self.B * input_u
        P_pred = self.A @ self.P_estimate @ self.A.T + self.Q

        # Update
        K = P_pred @ self.C.T @ np.linalg.inv(self.C @ P_pred @ self.C.T + self.R)
        self.x_estimate = x_pred + K @ (y_measure - self.C @ x_pred)
        self.P_estimate = (np.eye(2) - K @ self.C) @ P_pred
        # self.P_estimate = (1 - K @ self.C) @ P_pred # 이거는 1차원일때만 가능
        
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
