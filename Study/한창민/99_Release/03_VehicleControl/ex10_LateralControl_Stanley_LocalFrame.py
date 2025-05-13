import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat
from ex06_GlobalFrame2LocalFrame import Global2Local, PolynomialFitting, PolynomialValue

# ---------------------------------------
# Stanley Controller Class
# ---------------------------------------
class StanleyMethod(object):
    def __init__(self, K=1.0, L=2.5):
        self.K = K      # gain for cte
        self.L = L      # wheelbase
        self.u = 0.0    # steering angle (rad)

    def ControllerInput(self, coeff, yaw_error, Vx):
        # CTE: y at x=0
        cte = coeff[-1][0]

        # Stanley steering law
        delta_c = np.arctan2(self.K * cte, Vx)  # cross-track error term
        self.u = yaw_error + delta_c
        self.u = np.clip(self.u, -0.5, 0.5)  # steering angle limit

# ---------------------------------------
# Main Simulation
# ---------------------------------------
if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0
    num_degree = 3
    num_point = 5
    x_local = np.arange(0.0, 10.0, 0.5)

    # Reference Path (Global)
    X_ref = np.arange(0.0, 100.0, 0.1)
    Y_ref = 2.0 - 2.0 * np.cos(X_ref / 10.0)

    ego_vehicle = VehicleModel_Lat(step_time, Vx)
    frameconverter = Global2Local(num_point)
    polynomialfit = PolynomialFitting(num_degree, num_point)
    polynomialvalue = PolynomialValue(num_degree, len(x_local))
    controller = StanleyMethod(K=1.0, L=2.5)

    time = []
    X_ego = []
    Y_ego = []

    for i in range(int(simulation_time / step_time)):
        time.append(i * step_time)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)

        # Reference points around ego
        X_ref_convert = np.arange(ego_vehicle.X, ego_vehicle.X + 5.0, 1.0)
        Y_ref_convert = 2.0 - 2.0 * np.cos(X_ref_convert / 10.0)
        Points_ref = np.transpose(np.array([X_ref_convert, Y_ref_convert]))

        # Convert to Local frame and fit
        frameconverter.convert(Points_ref, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        polynomialfit.fit(frameconverter.LocalPoints)
        polynomialvalue.calculate(polynomialfit.coeff, x_local)

        # 경로 tangent 기울기 = dy/dx at x=0
        a = polynomialfit.coeff.flatten()
        dy = a[1] + 2 * a[2] * 0 + 3 * a[3] * 0**2
        yaw_path = np.arctan(dy)  # 경로 기울기
        yaw_error = -yaw_path     # 차량은 local yaw = 0이므로, 차이 = -경로각

        # Stanley Controller
        controller.ControllerInput(polynomialfit.coeff, yaw_error, Vx)

        # 차량 상태 갱신
        ego_vehicle.update(controller.u, Vx)

    # 시각화
    plt.figure(1)
    plt.plot(X_ref, Y_ref, 'k-', label="Reference")
    plt.plot(X_ego, Y_ego, 'b-', label="Ego Vehicle")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Stanley Method Path Tracking")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.show()
