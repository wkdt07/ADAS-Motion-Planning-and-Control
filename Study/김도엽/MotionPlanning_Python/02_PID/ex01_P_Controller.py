from vehicle_model import VehicleModel 
import numpy as np
import matplotlib.pyplot as plt

class P_Controller(object):
    def __init__(self, P_Gain=0.5):
        self.Kp = P_Gain
        self.u = 0.0  # 제어 입력 초기화
        #kp : 비례 제어 이득, 오차가 클 수록 더 큰 힘을 가함
        #u  : 제어 입력값. Fy, 즉 y 방향으로 가해질 힘을 나타냄

    def ControllerInput(self, reference, measure):
        error = reference - measure
        self.u = self.Kp * error 
        #reference : 원하는 위치
        #measure : 현재 위치
        #error : 오차
        #u = kp * error 오차가 클수록 더 큰 힘을 작용


if __name__ == "__main__":
    target_y = 0.0
    measure_y = []
    time = []
    step_time = 0.1
    simulation_time = 50   
    
    # 차량 초기 위치 y0 = 1.0, 속도 vx = 0.99, 질량 m = 1
    plant = VehicleModel(step_time, 0.0, 0.99, 0.1)
    controller = P_Controller(P_Gain=0.3)  # Gain은 실험적으로 조정 가능

    for i in range(int(simulation_time / step_time)):
        time.append(step_time * i)
        current_y = plant.y_measure[0][0]
        measure_y.append(current_y)
        controller.ControllerInput(target_y, current_y)
        plant.ControlInput(controller.u)

    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k--', label="Reference: y=0")
    plt.plot(time, measure_y, 'r-', label="Vehicle Position")
    plt.xlabel('Time (s)')
    plt.ylabel('Y Position')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
