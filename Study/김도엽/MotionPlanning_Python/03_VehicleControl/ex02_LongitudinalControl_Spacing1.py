import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Long import VehicleModel_Long

class PID_Controller_ConstantSpace(object):
    def __init__(self, step_time,
                       target_x, ego_x,
                       constantSpace=30.0,
                       P_Gain=0.5, I_Gain=0.1, D_Gain=0.05):
        self.dt      = step_time       # 샘플 시간
        self.space   = constantSpace   # 목표 간격 (m)
        # PID 게인
        self.Kp      = P_Gain
        self.Ki      = I_Gain
        self.Kd      = D_Gain
        # 상태 변수 초기화
        # 초기 오차는 (앞차-내차) - 목표간격
        init_err    = (target_x - ego_x) - self.space
        self.prev_e = init_err
        self.integral = 0.0
        self.u = 0.0
    
    def ControllerInput(self, target_x, ego_x):
        # 1) 현재 오차 계산
        e = (target_x - ego_x) - self.space
        
        # 2) 적분항 업데이트
        self.integral += e * self.dt
        
        # 3) 미분항 계산
        derivative = (e - self.prev_e) / self.dt
        
        # 4) PID 합산
        self.u = self.Kp * e \
               + self.Ki * self.integral \
               + self.Kd * derivative
        
        # 5) 상태 갱신
        self.prev_e = e
        
        # (return 은 선택사항, 속성 controller.u 로 읽어도 됨)
        return self.u


if __name__ == "__main__":
    
    step_time      = 0.1
    simulation_time= 50.0
    m              = 500.0
    
    vx_ego    = []
    vx_target = []
    x_space   = []
    time      = []
    
    # 앞차: 초기 위치 30m, 속도 10m/s
    target_vehicle = VehicleModel_Long(step_time, m, 0.0, 30.0, 10.0)
    # 내차: 초기 위치 0m, 속도 10m/s
    ego_vehicle    = VehicleModel_Long(step_time, m, 0.5, 0.0, 10.0)
    
    # PID 컨트롤러: 목표 간격 30m, 게인 P=0.5, I=0.1, D=0.05
    controller = PID_Controller_ConstantSpace(
        step_time,
        target_vehicle.x, ego_vehicle.x,
        constantSpace=30.0,
        P_Gain=0.12,
        I_Gain=0.008,
        D_Gain=0.225
    )
    
    for i in range(int(simulation_time/step_time)):
        t = step_time * i
        time.append(t)
        vx_ego.append(   ego_vehicle.vx)
        vx_target.append(target_vehicle.vx)
        x_space.append(  target_vehicle.x - ego_vehicle.x)
        
        # 제어입력 계산 & 적용
        controller.ControllerInput(target_vehicle.x, ego_vehicle.x)
        ego_vehicle.update(controller.u)
        target_vehicle.update(0.0)
        
    # 그래프 그리기 (중략, 원본과 동일)
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