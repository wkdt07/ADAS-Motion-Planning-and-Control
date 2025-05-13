import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Long import VehicleModel_Long

class PID_Controller_Speed(object):
    def __init__(self, reference, measure, step_time, P_Gain=7, D_Gain=0.3, I_Gain=0.0039):
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
    
    step_time = 0.1
    simulation_time = 50.0
    m = 500.0
    
    reference_speed = 30.0
    Vx = []
    ax = []
    time = []
    plant = VehicleModel_Long(step_time, m, 0.5, 0.0, 0.0)
    controller = PID_Controller_Speed(reference_speed, plant.vx, step_time)
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        Vx.append(plant.vx)
        ax.append(plant.ax)
        controller.ControllerInput(reference_speed,plant.vx)
        plant.update(controller.u)
        
    plt.figure(1)
    plt.plot(time, Vx,'r-',label = "Vx")
    plt.xlabel('time [s]')
    plt.ylabel('Vx [m/s]')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

    