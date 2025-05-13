import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat

class PID_Controller_Kinematic(object):
    def __init__(self, step_time, Y_ref, Y_ego, P_Gain=0.3, D_Gain=0.6, I_Gain=0.001):
        self.step_time = step_time
        self.Y_ref = Y_ref
        self.Y_ego = Y_ego
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.I_Gain = I_Gain

        self.pre_error = 0.0
        self.i_sum = 0.0
        self.u = 0.0  # 조향각 delta (rad)

    def ControllerInput(self, Y_ref, Y_ego):
        error = Y_ref - Y_ego
        d_error = (error - self.pre_error) / self.step_time
        self.i_sum += error * self.step_time

        # PID 제어식
        self.u = (self.P_Gain * error) + (self.D_Gain * d_error) + (self.I_Gain * self.i_sum)

        # 제한 걸기 (스티어링 한계 ±0.5 rad)
        self.u = np.clip(self.u, -0.5, 0.5)

        self.pre_error = error


if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0  # 종방향 속도 (정속)
    Y_ref = 4.0  # 목표 y 위치

    time = []
    X_ego = []
    Y_ego = []

    ego_vehicle = VehicleModel_Lat(step_time, Vx)
    controller = PID_Controller_Kinematic(step_time, Y_ref, ego_vehicle.Y)

    for i in range(int(simulation_time / step_time)):
        time.append(step_time * i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)

        controller.ControllerInput(Y_ref, ego_vehicle.Y)
        ego_vehicle.update(controller.u, Vx)

    plt.figure(1)
    plt.plot(X_ego, Y_ego, 'b-', label="Ego Path")
    plt.plot([0, X_ego[-1]], [Y_ref, Y_ref], 'r--', label="Reference Y = 4.0")
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Lateral Control (PID)')
    plt.legend(loc="best")
    plt.grid(True)
    plt.show()
