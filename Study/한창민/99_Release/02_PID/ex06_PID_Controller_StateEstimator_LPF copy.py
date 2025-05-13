from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=1 , D_Gain=4, I_Gain=0.1):
        self.u =0
        self.step_time = step_time
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.I_Gain = I_Gain
        self.pre_err = 0 
        self.i_sum = 0
    
    def ControllerInput(self, reference, measure):
        err = reference - measure        
        d_err = (err - self.pre_err) / self.step_time
        self.i_sum += self.step_time * err
        self.u = (self.P_Gain * err) + (self.D_Gain *d_err) + (self.I_Gain * self.i_sum)
        self.pre_err = err
        
class LowPassFilter:
    def __init__(self, y_initial_measure, alpha=0.9):
        self.y_estimate = y_initial_measure
        self.alpha = alpha
        
 
    def estimate(self, y_measure):
        self.y_estimate = self.alpha *self.y_estimate + (1- self.alpha)*y_measure



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
