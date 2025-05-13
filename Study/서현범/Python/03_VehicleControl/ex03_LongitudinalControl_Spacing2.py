import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Long import VehicleModel_Long


# timegap 기반 ACC
# 목표 거리차 = timegap * ego_vx
# 그래서 timegap을 유지하기 위해서 ego 차량의 속도를 조절하는 것
class PID_Controller_ConstantTimeGap(object):
    def __init__(self, step_time, target_x, ego_x, ego_vx, timegap = 2.0,  P_Gain=1.0, D_Gain=0.8, I_Gain=0.0039):
        self.timegap = timegap
        self.space = ego_vx * self.timegap  # 초기 목표 거리
        # Code
        self.dt = step_time
        self.Kp = P_Gain
        self.Kd = D_Gain
        self.Ki = I_Gain
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.u = 0.0  # 제어 입력 (가속도)
    def ControllerInput(self, target_x, ego_x, ego_vx):
        # Code
        self.space = ego_vx * self.timegap

        error = (target_x - ego_x) - self.space
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.u = self.Kp * error + self.Kd * derivative + self.Ki * self.integral
        self.prev_error = error
        

if __name__ == "__main__":
    
    step_time = 0.1
    simulation_time = 50.0
    m = 500.0
    
    vx_ego = []
    vx_target = []
    x_space = []
    x_reference = []
    timegap = []
    time = []
    target_vehicle = VehicleModel_Long(step_time, m, 0.0, 30.0, 10.0)
    ego_vehicle = VehicleModel_Long(step_time, m, 0.5, 0.0, 5.0)
    controller = PID_Controller_ConstantTimeGap(step_time, target_vehicle.x, ego_vehicle.x, ego_vehicle.vx)
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        vx_ego.append(ego_vehicle.vx)
        vx_target.append(target_vehicle.vx)
        x_space.append(target_vehicle.x - ego_vehicle.x)
        x_reference.append(controller.space)
        timegap.append((target_vehicle.x - ego_vehicle.x)/ego_vehicle.vx)
        controller.ControllerInput(target_vehicle.x, ego_vehicle.x, ego_vehicle.vx)
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
    plt.plot(time, x_reference,'k-',label = "reference space [m]")
    plt.plot(time, x_space,'b-',label = "space [m]")
    plt.xlabel('time [s]')
    plt.ylabel('x')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    
    plt.figure(3)
    plt.plot([0, time[-1]], [controller.timegap, controller.timegap],'k-',label = "reference timegap [s]")
    plt.plot(time, timegap,'b-',label = "timegap [s]")
    plt.xlabel('time [s]')
    plt.ylabel('x')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)    
    
    plt.show()

    