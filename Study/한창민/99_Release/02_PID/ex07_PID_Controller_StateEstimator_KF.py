from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=0.3, D_Gain=0.9, I_Gain=0.01):
        self.u = 0.0
        self.step_time = step_time
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.I_Gain = I_Gain
        self.pre_err = 0.0
        self.i_sum = 0.0
    
    def ControllerInput(self, reference, measure):
        err = reference - measure        
        d_err = (err - self.pre_err) / self.step_time
        self.i_sum += self.step_time * err
        self.u = (self.P_Gain * err) + (self.D_Gain * d_err) + (self.I_Gain * self.i_sum)
        self.pre_err = err

class KalmanFilter:
    def __init__(self, x0=0.0, step_time=0.1, m=1.0, q=0.01, r=0.05, p0=1.0):
        self.dt = step_time
        self.A = 1.0
        self.B = self.dt / m
        self.C = 1.0

        self.Q = q
        self.R = r

        self.x = x0
        self.P = p0

    def estimate(self, measurement, u):
        # Prediction
        x_pred = self.A * self.x + self.B * u
        P_pred = self.A * self.P * self.A + self.Q

        # Kalman Gain
        K = P_pred * self.C / (self.C * P_pred * self.C + self.R)

        # Correction
        self.x = x_pred + K * (measurement - self.C * x_pred)
        self.P = (1 - K * self.C) * P_pred

        return self.x

if __name__ == "__main__":
    target_y = 0.0
    measure_y = []
    estimated_y = []
    time = []
    step_time = 0.1
    simulation_time = 30

    plant = VehicleModel(step_time, 0.25, 0.99, 0.05)
    estimator = KalmanFilter(x0=plant.y_measure[0][0], step_time=step_time, m=1.0, q=0.01, r=0.05, p0=1.0)
    controller = PID_Controller(target_y, plant.y_measure[0][0], step_time)

    for i in range(int(simulation_time / step_time)):
        time.append(step_time * i)
        current_measure = plant.y_measure[0][0]
        measure_y.append(current_measure)

        est_y = estimator.estimate(current_measure, controller.u)
        estimated_y.append(est_y)

        controller.ControllerInput(target_y, est_y)
        plant.ControlInput(controller.u)

    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="Reference")
    plt.plot(time, measure_y, 'r:', label="Measured Position")
    plt.plot(time, estimated_y, 'c-', label="Kalman Estimated Position")
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
