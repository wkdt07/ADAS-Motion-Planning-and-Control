import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat
from ex06_GlobalFrame2LocalFrame import Global2Local
from ex06_GlobalFrame2LocalFrame import PolynomialFitting
from ex06_GlobalFrame2LocalFrame import PolynomialValue

    
if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0
    X_ref = np.arange(0.0, 100.0, 0.1)
    Y_ref = 2.0-2*np.cos(X_ref/10)
    num_degree = 3
    num_point = 5
    x_local = np.arange(0.0, 10.0, 0.5)

    class StanleyMethod(object):
        def __init__(self, step_time, coeff, Vx, k=10, wheelbase=2.5):
            # Code
            self.k = k
            self.L = wheelbase
            self.epsilon = 1e-5  # 0 나눗셈 방지
            self.u = 0.0
        def ControllerInput(self,coeff, Vx):
            # Code
            a0 = coeff[3][0]  # lateral error (y offset at x=0)
            a1 = coeff[2][0]  # path slope at x=0 (f'(0) = a1)

            # 1. 경로와 차량 heading 차이
            theta_path = np.arctan(a1)     # 경로 방향 (기울기 기준)
            theta_vehicle = 0.0            # Local Frame 기준 → 항상 0

            heading_error = theta_path - theta_vehicle

            # 2. CTE 기반 보정 (속도 영향 포함)
            cte_term = np.arctan2(self.k * a0, Vx + self.epsilon)

            # 3. 최종 조향각
            self.u = heading_error + cte_term
    time = []
    X_ego = []
    Y_ego = []
    ego_vehicle = VehicleModel_Lat(step_time, Vx)

    frameconverter = Global2Local(num_point)
    polynomialfit = PolynomialFitting(num_degree,num_point)
    polynomialvalue = PolynomialValue(num_degree,np.size(x_local))
    controller = StanleyMethod(step_time, polynomialfit.coeff, Vx ,)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        X_ego.append(float(ego_vehicle.X))
        Y_ego.append(ego_vehicle.Y)
        X_ref_convert = np.arange(float(ego_vehicle.X), float(ego_vehicle.X)+5.0, 1.0)
        Y_ref_convert = 2.0-2*np.cos(X_ref_convert/10)
        Points_ref = np.transpose(np.array([X_ref_convert, Y_ref_convert]))
        frameconverter.convert(Points_ref, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        polynomialfit.fit(frameconverter.LocalPoints)
        polynomialvalue.calculate(polynomialfit.coeff, x_local)
        controller.ControllerInput(polynomialfit.coeff, Vx)
        ego_vehicle.update(controller.u, Vx)

        
    plt.figure(1)
    plt.plot(X_ref, Y_ref,'k-',label = "Reference")
    plt.plot(X_ego, Y_ego,'b-',label = "Position")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
#    plt.axis("best")
    plt.grid(True)    
    plt.show()


