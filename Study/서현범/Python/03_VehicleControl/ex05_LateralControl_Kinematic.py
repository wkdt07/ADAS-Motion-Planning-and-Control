import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat

# 차량 횡방향 제어
# 차량의 조향각(steering angle)을 어떻게 선택할까?

class PID_Controller_Kinematic(object):
    def __init__(self, step_time, y_ref, y_init,
                 P_Gain=1.7, D_Gain=1, I_Gain=0.038):
        # 제어 주기
        self.dt    = step_time
        # 목표 경로 y = y_ref
        self.y_ref = y_ref
        
        # PID 이득
        self.Kp    = P_Gain
        self.Kd    = D_Gain
        self.Ki    = I_Gain
        
        # 초기 오차, 적분항
        self.prev_error = y_ref - y_init
        self.integral   = 0.0
        
        # 출력(조향각)
        self.u = 0.0
    def ControllerInput(self, y_ref, y):
        # 1) 오차 계산
        error = y_ref - y
        
        # 2) 적분 누적
        self.integral += error * self.dt
        
        # 3) 미분항
        derivative = (error - self.prev_error) / self.dt
        
        # 4) PID 계산 → 조향각
        self.u = ( self.Kp * error
                 + self.Ki * self.integral
                 + self.Kd * derivative )
        
        # 5) 상태 업데이트
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


