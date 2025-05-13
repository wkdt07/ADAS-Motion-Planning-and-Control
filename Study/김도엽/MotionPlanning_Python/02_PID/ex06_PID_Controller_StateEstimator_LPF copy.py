from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=0.4, D_Gain=0.9, I_Gain=0.02):
        self.Kp = P_Gain
        self.Kd = D_Gain
        self.Ki = I_Gain
        self.step_time = step_time

        self.prev_error = reference - measure
        self.integral_error = 0.0
        self.u = 0.0

    def ControllerInput(self, reference, measure):
        error = reference - measure
        self.integral_error += error * self.step_time
        d_error = (error - self.prev_error) / self.step_time
        self.u = self.Kp * error + self.Ki * self.integral_error + self.Kd * d_error
        self.prev_error = error

class LowPassFilter:
    def __init__(self, init_val, alpha=0.2):
        self.alpha = alpha
        self.y_estimate = init_val

    def estimate(self, measurement):
        self.y_estimate = self.alpha * measurement + (1 - self.alpha) * self.y_estimate

if __name__ == "__main__":
    target_y = 0.0
    measure_y = []
    estimated_y = []
    output_raw = []
    output_filtered = []
    time = []
    step_time = 0.1
    simulation_time = 30   

    plant_raw = VehicleModel(step_time, 0.25, 0.99, 0.05)      # For raw measurement PID
    plant_filtered = VehicleModel(step_time, 0.25, 0.99, 0.05)  # For filtered PID

    estimator = LowPassFilter(plant_filtered.y_measure[0][0])
    controller_raw = PID_Controller(target_y, plant_raw.y_measure[0][0], step_time)
    controller_filtered = PID_Controller(target_y, plant_filtered.y_measure[0][0], step_time)

    for i in range(int(simulation_time / step_time)):
        t = step_time * i
        time.append(t)

        # Raw measurement (no filter)
        y_raw = plant_raw.y_measure[0][0]
        measure_y.append(y_raw)
        controller_raw.ControllerInput(target_y, y_raw)
        plant_raw.ControlInput(controller_raw.u)
        output_raw.append(y_raw)

        # Filtered measurement
        y_measured = plant_filtered.y_measure[0][0]
        estimator.estimate(y_measured)
        estimated_y.append(estimator.y_estimate)
        controller_filtered.ControllerInput(target_y, estimator.y_estimate)
        plant_filtered.ControlInput(controller_filtered.u)
        output_filtered.append(estimator.y_estimate)

    # Plot results
    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k--', label="Reference: y=0")
    plt.plot(time, measure_y, 'r-', label="Raw Measurement (No Filter)")
    plt.plot(time, estimated_y, 'c-', label="Filtered Estimate (LPF)")
    plt.xlabel('Time (s)')
    plt.ylabel('Y Position')
    plt.title("PID Comparison: Raw vs Filtered Measurement")
    plt.legend(loc="best")
    plt.grid(True)
    plt.axis("equal")
    plt.show()
