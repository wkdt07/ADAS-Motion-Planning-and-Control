import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Long import VehicleModel_Long

class PID_Controller_ConstantSpace(object):
    def __init__(self, step_time, target_x, ego_x, constantSpace=30.0, P_Gain=3, D_Gain=1, I_Gain=0.005):
        self.space = constantSpace
        self.step_time = step_time
        self.target_x = target_x
        self.ego_x = ego_x
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.I_Gain = I_Gain
        self.pre_error = 0.0
        self.i_sum = 0.0
        self.u = 0.0

    def ControllerInput(self, target_x, ego_x):
        error = (target_x - ego_x) - self.space  # 목표 간격과의 차이
        d_error = (error - self.pre_error) / self.step_time
        self.i_sum += error * self.step_time
        self.u = (self.P_Gain * error) + (self.D_Gain * d_error) + (self.I_Gain * self.i_sum)
        self.pre_error = error


if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 50.0
    m = 500.0

    vx_ego = []
    vx_target = []
    x_space = []
    time = []

    # target vehicle: 등속
    target_vehicle = VehicleModel_Long(step_time, m, 0.0, 30.0, 10.0)
    # ego vehicle: 추종 제어 대상
    ego_vehicle = VehicleModel_Long(step_time, m, 0.5, 0.0, 10.0)

    # controller: 목표 간격 30m
    controller = PID_Controller_ConstantSpace(step_time, target_vehicle.x, ego_vehicle.x, constantSpace=30.0, P_Gain=2.0, D_Gain=0.5, I_Gain=0.1)

    for i in range(int(simulation_time / step_time)):
        time.append(step_time * i)
        vx_ego.append(ego_vehicle.vx)
        vx_target.append(target_vehicle.vx)
        x_space.append(target_vehicle.x - ego_vehicle.x)
        controller.ControllerInput(target_vehicle.x, ego_vehicle.x)
        ego_vehicle.update(controller.u)
        target_vehicle.update(0.0)

    plt.figure(1)
    plt.plot(time, vx_ego, 'r-', label="ego_vx [m/s]")
    plt.plot(time, vx_target, 'b-', label="target_vx [m/s]")
    plt.xlabel('time [s]')
    plt.ylabel('Vx [m/s]')
    plt.legend(loc="best")
    plt.grid(True)
    plt.title("Velocity Profile")

    plt.figure(2)
    plt.plot([0, time[-1]], [controller.space, controller.space], 'k--', label="reference spacing")
    plt.plot(time, x_space, 'b-', label="actual spacing [m]")
    plt.xlabel('time [s]')
    plt.ylabel('Spacing [m]')
    plt.legend(loc="best")
    plt.grid(True)
    plt.title("Spacing Between Vehicles")

    plt.show()
