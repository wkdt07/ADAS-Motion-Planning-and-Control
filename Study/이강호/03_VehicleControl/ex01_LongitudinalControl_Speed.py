import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Long import VehicleModel_Long

class PID_Controller_Speed(object):
    def __init__(self, reference, measure, step_time, P_Gain=10.0, D_Gain=50.0, I_Gain=0.002):
        # Code
        self.target_Vx = reference
        self.Vx_measure = measure
        self.dt = step_time
        self.Kp = P_Gain
        self.Ki = I_Gain
        self.Kd = D_Gain

        self.error = reference - measure
        self.prev_error = 0
        self.i_err = 0

    def ControllerInput(self, reference, measure):
        # Code
        self.target_Vx = reference
        self.Vx_measure = measure
        self.error = reference - measure
        self.error_Gap = self.error - self.prev_error
        self.i_err += self.error
        self.prev_error = self.error

        self.P_Controller = self.Kp * self.error
        self.I_Controller = self.Ki * self.i_err
        self.D_Controller = self.Kd * self.error_Gap
        
        self.u = self.P_Controller + self.I_Controller + self.D_Controller

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

    