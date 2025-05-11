from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class P_Controller(object):
    def __init__(self, P_Gain=0.5):
        # Code
        self.P_Gain = P_Gain
        self.u = 0.0
        self.previous_error = 0.0
        
    def ControllerInput(self, reference, measure):
        # Code
        e = reference - measure
        self.u = self.P_Gain * e
        


if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    time = []
    step_time = 0.1
    simulation_time = 30   
    plant = VehicleModel(step_time, 0.0, 0.99, 0.1) # 이건 VehicleModel.py에 있는 VehicleModel class를 사용한 것임
    controller = P_Controller() # 여기서 control 실행
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        controller.ControllerInput(target_y, plant.y_measure[0][0])
        plant.ControlInput(controller.u) # 여기서 F 주면 그에 따라 y 조정
    
    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r-',label = "Vehicle Position")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
