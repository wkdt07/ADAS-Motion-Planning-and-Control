import numpy as np
import matplotlib.pyplot as plt

class VehicleModel_Lat(object):
    def __init__(self, dt, Vx):
        self.dt = dt        # 시간 간격
        self.Vx = Vx       # 속도
        self.X = 0.0       # 초기 x 위치
        self.Y = 0.0       # 초기 y 위치
        self.delta = 0.0   # 초기 steering angle

    def update(self, delta, Vx):
        self.delta = delta
        # 차량의 새로운 위치 계산
        self.X += Vx * np.cos(self.delta) * self.dt
        self.Y += Vx * np.sin(self.delta) * self.dt

class PID_Controller_Kinematic(object):
    def __init__(self, step_time, Y_ref, initial_Y):
        self.Kp = 5.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.previous_error = Y_ref - initial_Y
        self.integral = 0
        self.u = 0.0

    def ControllerInput(self, Y_ref, current_Y):
        # 오차 계산
        error = Y_ref - current_Y
        self.integral += error
        derivative = error - self.previous_error

        # PID 제어기 계산
        self.u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # 이전 오차 업데이트
        self.previous_error = error

if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0  # 차량 속도 (m/s)
    Y_ref = 4.0  # 레퍼런스 y 값

    time = []
    X_ego = []
    Y_ego = []
    
    ego_vehicle = VehicleModel_Lat(step_time, Vx)  # 차량 모델 객체 생성
    controller = PID_Controller_Kinematic(step_time, Y_ref, ego_vehicle.Y)  # PID 제어기 객체 생성
    
    # 시뮬레이션 루프
    for i in range(int(simulation_time / step_time)):
        time.append(step_time * i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)
        
        controller.ControllerInput(Y_ref, ego_vehicle.Y)  # PID 제어기 입력
        ego_vehicle.update(controller.u, Vx)  # 차량 위치 업데이트

    # 결과 출력
    plt.figure(1)
    plt.plot(X_ego, Y_ego, 'b-', label="Position")
    plt.plot([0, X_ego[-1]], [Y_ref, Y_ref], 'k:', label="Reference")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
    plt.grid(True)
    plt.show()
