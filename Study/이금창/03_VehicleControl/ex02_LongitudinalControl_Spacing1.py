import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Long import VehicleModel_Long

class PID_Controller_ConstantSpace(object):
    def __init__(self, step_time, target_x, ego_x, constantSpace=30.0, P_Gain=0.45, D_Gain=0.45, I_Gain=0.018):
        self.space = constantSpace  # 목표 간격
        self.Kp = P_Gain
        self.Kd = D_Gain
        self.Ki = I_Gain
        self.dt = step_time

        self.prev_error = target_x - ego_x
        self.integral_error = 0.0
        self.u = 0.0  # 제어 입력 (가속도나 속도 조정)

    def ControllerInput(self, target_x, ego_x):
        # 목표 거리와 현재 거리 차이 계산
        error = target_x - ego_x - self.space  # 목표 차이에서 일정 공간만큼 차감
        self.integral_error += error * self.dt  # 적분 오차 누적
        d_error = (error - self.prev_error) / self.dt  # 미분 오차

        # PID 제어 공식
        self.u = self.Kp * error + self.Ki * self.integral_error + self.Kd * d_error
        self.prev_error = error  # 이전 오차 저장


        

if __name__ == "__main__":
    
    step_time = 0.1
    simulation_time = 50.0
    m = 500.0
    
    vx_ego = []
    vx_target = []
    x_space = []
    time = []
    target_vehicle = VehicleModel_Long(step_time, m, 0.0, 30.0, 10.0)
    ego_vehicle = VehicleModel_Long(step_time, m, 0.5, 0.0, 10.0)
    controller = PID_Controller_ConstantSpace(step_time, target_vehicle.x, ego_vehicle.x)
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        vx_ego.append(ego_vehicle.vx)
        vx_target.append(target_vehicle.vx)
        x_space.append(target_vehicle.x-ego_vehicle.x)
        controller.ControllerInput(target_vehicle.x, ego_vehicle.x)
        ego_vehicle.update(controller.u)
        target_vehicle.update(0.0)
        
        
    plt.figure(1)
    plt.plot(time, vx_ego,'r-',label = "ego_vx [m/s]")
    plt.plot(time, vx_target,'b-',label = "target_vx [m/s]")
    plt.xlabel('time [s]')
    plt.ylabel('Vx')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    
    plt.figure(2)
    plt.plot([0, time[-1]], [controller.space, controller.space], 'k-', label="reference")
    plt.plot(time, x_space,'b-',label = "space [m]")
    plt.xlabel('time [s]')
    plt.ylabel('x')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)    
    
    plt.show()

    