import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat

class PID_Controller_Kinematic(object):
    def __init__(self, dt, reference, measure, P_Gain=0.35, I_Gain=0.0005, D_Gain=0.7525):
        """ # 54
        dt         : 제어 주기 (step_time)
        reference  : 목표 Y 위치
        measure    : 초기 Y 측정값
        P_Gain     : 비례 이득
        I_Gain     : 적분 이득
        D_Gain     : 미분 이득
        """
        self.dt        = dt
        self.target_y  = reference
        self.y_measure = measure

        self.Kp = P_Gain
        self.Ki = I_Gain
        self.Kd = D_Gain

        self.i_err     = 0.0
        self.prev_error= 0.0
        self.u         = 0.0

    def ControllerInput(self, reference, measure):
        """
        reference: 목표 Y 위치
        measure  : 현재 Y 위치
        return   : 계산된 조향각(delta)
        """
        # 오차 계산
        error = reference - measure

        # 적분/미분 계산
        self.i_err    += error * self.dt
        derivative    = (error - self.prev_error) / self.dt

        # PID 연산
        P = self.Kp * error
        I = self.Ki * self.i_err
        D = self.Kd * derivative

        # 제어 입력(delta) 계산 및 저장
        self.u = P + I + D
        self.prev_error = error

if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0
    Y_ref = 4.0
    
    time = []
    X_ego = []
    Y_ego = []
    ego_vehicle = VehicleModel_Lat(step_time, Vx)
    controller = PID_Controller_Kinematic(step_time, Y_ref, ego_vehicle.Y)
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)
        controller.ControllerInput(Y_ref, ego_vehicle.Y)
        ego_vehicle.update(controller.u, Vx)

        
    plt.figure(1)
    plt.plot(X_ego, Y_ego,'b-',label = "Position")
    plt.plot([0, X_ego[-1]], [Y_ref, Y_ref], 'k:',label = "Reference")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
#    plt.axis("best")
    plt.grid(True)    
    plt.show()


