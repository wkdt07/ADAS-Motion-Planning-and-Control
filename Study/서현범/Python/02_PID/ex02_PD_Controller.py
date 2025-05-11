from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PD_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=0.6, D_Gain=1.2):
        # Code
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.step_time = step_time
        self.previous_error = 0.0
        self.u = 0.0
        self.e_prev = 0.0

    def ControllerInput(self, reference, measure):
        # Code
        '''
        이제 PD 제어기를 쓰는데, D 제어기는 미분 제어기 -> 이전 값과 함께 해야함.
        이산적으로 치면,
        u = Kp * e + Kd * (e - e_prev) / dt
        e_prev = e
        '''

        e = reference - measure
        self.u = self.P_Gain * e + self.D_Gain * (e - self.e_prev) / self.step_time
        self.e_prev = e # 이전 에러를 저장해둠
        


if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    time = []
    step_time = 0.1
    simulation_time = 30   
    plant = VehicleModel(step_time, 0.0, 0.99, 0.1)
    controller = PD_Controller(target_y, plant.y_measure[0][0], step_time)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        controller.ControllerInput(target_y, plant.y_measure[0][0])
        plant.ControlInput(controller.u)
    
    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r-',label = "Vehicle Position")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
