from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=0.48, D_Gain=2.3, I_Gain=0.08):
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

if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    time = []
    step_time = 0.1
    simulation_time = 30   
    plant = VehicleModel(step_time, 0.0, 0.4, -0.1)
    controller = PID_Controller(target_y, plant.y_measure[0][0], step_time)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        controller.ControllerInput(target_y, plant.y_measure[0][0])
        plant.ControlInput(controller.u)

    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r-',label = "Vehicle Position")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
