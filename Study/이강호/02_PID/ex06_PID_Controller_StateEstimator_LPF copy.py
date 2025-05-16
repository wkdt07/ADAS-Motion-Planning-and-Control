from vehicle_model import VehicleModel
from numpy.linalg import inv
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=10.0, D_Gain=50.0, I_Gain=0.02):
        # Code
        self.target_y = reference
        self.y_measure = measure
        self.dt = step_time
        self.Kp = P_Gain
        self.Ki = I_Gain
        self.Kd = D_Gain

        self.error = reference - measure
        self.prev_error = 0
        self.i_err = 0

    def ControllerInput(self, reference, measure):
        # Code
        self.target_y = reference
        self.y_measure = measure
        self.error = reference - measure
        self.error_Gap = self.error - self.prev_error
        self.i_err += self.error
        self.prev_error = self.error
        self.u = self.Kp * self.error + self.Ki * self.i_err + self.Kd * self.error_Gap

class LowPassFilter:
    def __init__(self, y_initial_measure, alpha=0.2):
        self.y_initial_measure = y_initial_measure
        self.alpha = alpha
        self.y_estimate = y_initial_measure
        self.i = 0.0

    def estimate(self, y_measure):
        if self.i == 0.0:
            value = self.y_initial_measure
            self.i += 1
        else:
            value = self.y_initial_measure * self.alpha + (1 - self.alpha) * y_measure 
        self.y_estimate = value
        return value
if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    estimated_y = []
    time = []
    step_time = 0.1
    simulation_time = 30   
    plant = VehicleModel(step_time, 0.25, 0.99, 0.05)
    estimator = LowPassFilter(plant.y_measure[0][0])
    controller = PID_Controller(target_y, plant.y_measure[0][0], step_time)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        estimated_y.append(estimator.y_estimate)
        estimator.estimate(plant.y_measure[0][0])
        controller.ControllerInput(target_y, estimator.y_estimate)
        plant.ControlInput(controller.u)
    
    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r-',label = "Vehicle Position(Measure)")
    plt.plot(time, estimated_y,'c-',label = "Vehicle Position(Estimator)")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
