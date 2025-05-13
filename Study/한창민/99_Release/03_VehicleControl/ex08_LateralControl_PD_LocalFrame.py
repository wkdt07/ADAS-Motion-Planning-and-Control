import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat
from ex06_GlobalFrame2LocalFrame import Global2Local, PolynomialFitting, PolynomialValue

# --------------------------------------------
# PD + Feedforward Controller for High-Speed
# --------------------------------------------
class PD_Controller(object):
    def __init__(self, step_time, P_Gain=3.0, D_Gain=0.5, L=2.5):
        self.step_time = step_time
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.L = L  # wheelbase
        self.prev_error = 0.0
        self.u = 0.0  # steering angle (rad)

    def ControllerInput(self, cte, curvature, Vx):
        # PD feedback
        d_error = (cte - self.prev_error) / self.step_time
        feedback = self.P_Gain * cte + self.D_Gain * d_error

        # Feedforward using curvature
        feedforward = np.arctan(self.L * curvature)

        # Total steering angle
        self.u = feedback + feedforward
        self.u = np.clip(self.u, -0.5, 0.5)  # steering limit
        self.prev_error = cte

# --------------------------------------------
# Main simulation
# --------------------------------------------
if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 20.0  # 고속 설정
    num_degree = 3
    num_point = 5
    x_local = np.arange(0.0, 10.0, 0.5)

    # Global reference path
    X_ref = np.arange(0.0, 100.0, 0.1)
    Y_ref = 2.0 - 2.0 * np.cos(X_ref / 10.0)

    # 객체 생성
    ego_vehicle = VehicleModel_Lat(step_time, Vx)
    frameconverter = Global2Local(num_point)
    polynomialfit = PolynomialFitting(num_degree, num_point)
    polynomialvalue = PolynomialValue(num_degree, len(x_local))
    controller = PD_Controller(step_time)

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

        # Local 변환 + 다항식 피팅
        frameconverter.convert(Points_ref, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        polynomialfit.fit(frameconverter.LocalPoints)
        polynomialvalue.calculate(polynomialfit.coeff, x_local)

        # CTE 및 곡률 계산
        cte = polynomialvalue.points[0][1]
        a = polynomialfit.coeff.flatten()
        dy = a[1] + 2 * a[2] * 0 + 3 * a[3] * 0**2
        ddy = 2 * a[2] + 6 * a[3] * 0
        curvature = ddy / ((1 + dy**2) ** 1.5)

        # 제어 입력
        controller.ControllerInput(cte, curvature, Vx)

        # 차량 갱신
        ego_vehicle.update(controller.u, Vx)

    # 시각화
    plt.figure(1)
    plt.plot(X_ref, Y_ref, 'k-', label="Reference")
    plt.plot(X_ego, Y_ego, 'b-', label="Ego Position")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
    plt.title("High-Speed Lateral Control with Feedforward")
    plt.grid(True)
    plt.axis("equal")
    plt.show()
