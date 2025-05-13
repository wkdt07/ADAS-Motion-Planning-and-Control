import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Long import VehicleModel_Long

class PID_Controller_Speed(object):
    def __init__(self, reference, measure, step_time, P_Gain=1.5, D_Gain=0.1, I_Gain=0.005):
        self.step_time = step_time
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.I_Gain = I_Gain
        self.pre_error = 0.0
        self.i_sum = 0.0
        self.u = 0.0  # Output control (acceleration force)

    def ControllerInput(self, reference, measure):
        error = reference - measure
        d_error = (error - self.pre_error) / self.step_time
        self.i_sum += error * self.step_time

        # PID 제어 식
        self.u = (self.P_Gain * error) + (self.D_Gain * d_error) + (self.I_Gain * self.i_sum)

        self.pre_error = error


if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 50.0
    m = 500.0  # 차량 질량 [kg]

    reference_speed = 30.0  # 목표 속도 [m/s]
    Vx = []
    ax = []
    time = []

    plant = VehicleModel_Long(step_time, m, 0.5, 0.0, 0.0)  # VehicleModel_Long: F = ma 기반 모델
    controller = PID_Controller_Speed(reference_speed, plant.vx, step_time)

    for i in range(int(simulation_time / step_time)):
        time.append(step_time * i)
        Vx.append(plant.vx)
        ax.append(plant.ax)
        controller.ControllerInput(reference_speed, plant.vx)
        plant.update(controller.u)

    plt.figure(1)
    plt.plot(time, Vx, 'r-', label="Vx")
    plt.xlabel('time [s]')
    plt.ylabel('Vx [m/s]')
    plt.legend(loc="best")
    plt.grid(True)
    plt.title("PID Speed Control Result")
    plt.show()
