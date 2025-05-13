import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat
from ex06_GlobalFrame2LocalFrame import Global2Local, PolynomialFitting, PolynomialValue

# ------------------------------------------
# Pure Pursuit 제어기 클래스
# ------------------------------------------
class PurePursuit(object):
    def __init__(self, L=2.5, Ld=3.0):
        self.L = L      # 휠베이스
        self.Ld = Ld    # Lookahead 거리
        self.u = 0.0    # 조향각 (rad)

    def ControllerInput(self, local_path):
        for i in range(len(local_path)):
            x = local_path[i][0]
            y = local_path[i][1]
            dist = np.sqrt(x**2 + y**2)
            if dist >= self.Ld:
                alpha = np.arctan2(y, x)
                self.u = np.arctan2(2 * self.L * np.sin(alpha), self.Ld)
                return
        self.u = 0.0  # 못 찾으면 직진

# ------------------------------------------
# 시뮬레이션 설정 및 실행
# ------------------------------------------
if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0  # 자차 속도 (m/s)
    num_degree = 3
    num_point = 5
    x_local = np.arange(0.0, 10.0, 0.5)

    # Global reference path 정의
    X_ref = np.arange(0.0, 100.0, 0.1)
    Y_ref = 2.0 - 2.0 * np.cos(X_ref / 10.0)

    # 객체 생성
    ego_vehicle = VehicleModel_Lat(step_time, Vx)
    frameconverter = Global2Local(num_point)
    polynomialfit = PolynomialFitting(num_degree, num_point)
    polynomialvalue = PolynomialValue(num_degree, len(x_local))
    controller = PurePursuit(L=2.5, Ld=3.0)

    # 기록용 리스트
    time = []
    X_ego = []
    Y_ego = []

    for i in range(int(simulation_time / step_time)):
        time.append(i * step_time)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)

        # 앞쪽 global reference 잘라오기
        X_ref_convert = np.arange(ego_vehicle.X, ego_vehicle.X + 5.0, 1.0)
        Y_ref_convert = 2.0 - 2.0 * np.cos(X_ref_convert / 10.0)
        Points_ref = np.transpose(np.array([X_ref_convert, Y_ref_convert]))

        # Local Frame 변환 + 다항식 피팅
        frameconverter.convert(Points_ref, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        polynomialfit.fit(frameconverter.LocalPoints)
        polynomialvalue.calculate(polynomialfit.coeff, x_local)

        # Pure Pursuit 제어 입력
        controller.ControllerInput(polynomialvalue.points)

        # 차량 갱신
        ego_vehicle.update(controller.u, Vx)

    # 시각화
    plt.figure(1)
    plt.plot(X_ref, Y_ref, 'k-', label="Reference")
    plt.plot(X_ego, Y_ego, 'b-', label="Ego Position")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title("Pure Pursuit Path Tracking")
    plt.legend(loc="best")
    plt.grid(True)
    plt.axis("equal")
    plt.show()
