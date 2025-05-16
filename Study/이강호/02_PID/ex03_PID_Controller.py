#!/usr/bin/env python3

from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=10.0, D_Gain=50.0, I_Gain=0.02):
        self.target_y = reference
        self.y_measure = measure
        self.step_time = step_time
        self.error = reference-measure
        self.errorSum = 0
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.I_Gain = I_Gain

    def ControllerInput(self, reference, measure):
        self.target_y = reference
        self.y_measure = measure

        #errorGap for D
        self.errorGap = (reference-measure) - self.error

        #update error
        self.error = reference-measure

        #integral for I
        self.errorSum+= self.error
        #print(self.errorSum)
        self.p_controller = self.P_Gain * self.error
        self.d_controller = self.D_Gain * self.errorGap
        self.i_controller = self.I_Gain * self.errorSum

        self.u = self.p_controller + self.d_controller +self.i_controller

if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    time = []
    step_time = 0.1
    simulation_time = 30
    plant = VehicleModel(step_time, 0.0, 0.99, 0.05)
    controller = PID_Controller(target_y, plant.y_measure[0][0], step_time)

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
    #plt.axis("equal")
    plt.grid(True)
    plt.show()