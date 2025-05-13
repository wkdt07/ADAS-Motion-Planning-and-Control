import numpy as np
import matplotlib.pyplot as plt
from lane_1 import lane

#### Polynomial value calculation
def Polyval(coeff, x):
    """
    Evaluate a polynomial with given coefficients at x.
    coeff: array-like, polynomial coefficients [a_n, a_{n-1}, ..., a_0]
    x: scalar or array-like, points at which to evaluate the polynomial
    Returns: polynomial value(s) at x
    """
    return np.polyval(coeff, x)

# Global coordinate --> Local coordinate
def Global2Local(global_points, yaw_ego, X_ego, Y_ego):
    """
    Transform points from global coordinates to vehicle's local coordinate frame.
    global_points: list of [X_global, Y_global] points
    yaw_ego: heading angle of ego vehicle (rad)
    X_ego, Y_ego: global position of ego vehicle
    Returns: list of [x_local, y_local] points
    """
    local_points = []
    cos_yaw = np.cos(yaw_ego)
    sin_yaw = np.sin(yaw_ego)
    for Xg, Yg in global_points:
        dx = Xg - X_ego
        dy = Yg - Y_ego
        x_local = dx * cos_yaw + dy * sin_yaw
        y_local = -dx * sin_yaw + dy * cos_yaw
        local_points.append([x_local, y_local])
    return local_points

# Polynomial fitting (n_th order)
def Polyfit(points, num_order):
    """
    Fit a polynomial of order num_order to given 2D points.
    points: list of [x, y] points in local frame
    num_order: order of polynomial to fit
    Returns: array of polynomial coefficients [a_n, ..., a_0]
    """
    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])
    coeff = np.polyfit(xs, ys, num_order)
    return coeff

# Both lane to path
def BothLane2Path(coeff_L, coeff_R):
    """
    Compute the center path polynomial from left and right lane polynomials.
    coeff_L, coeff_R: polynomial coefficients for left and right lanes [a_n, ..., a_0]
    Returns: coefficients for center path: (coeff_L + coeff_R) / 2
    """
    coeff_L = np.array(coeff_L)
    coeff_R = np.array(coeff_R)
    coeff_center = (coeff_L + coeff_R) / 2.0
    return coeff_center

# Vehicle model
defaults = {'m':500, 'L':4.0, 'kv':0.005}
class VehicleModel_Lat(object):
    def __init__(self, step_time, Vx, m=defaults['m'], L=defaults['L'], kv=defaults['kv'], Pos=[0.0,0.0,0.0]):
        """
        Lateral vehicle model using bicycle approximation.
        step_time: time step dt
        Vx: longitudinal speed
        m: mass
        L: wheelbase
        kv: velocity dependent factor
        Pos: initial [X, Y, Yaw]
        """
        self.dt = step_time
        self.m = m
        self.L = L
        self.kv = kv
        self.vx = Vx
        self.delta = 0.0
        self.yawrate = 0.0
        self.Yaw = Pos[2]
        self.X = Pos[0]
        self.Y = Pos[1]

    def update(self, delta, Vx):
        """
        Update vehicle state given steering angle delta and speed Vx.
        delta: steering input (rad)
        Vx: longitudinal speed
        """
        self.vx = Vx
        # limit steering angle
        self.delta = np.clip(delta, -0.5, 0.5)
        # bicycle model yaw rate
        self.yawrate = self.vx / (self.L + self.kv * self.vx**2) * self.delta
        # integrate yaw
        self.Yaw += self.dt * self.yawrate
        # update position
        self.X += Vx * self.dt * np.cos(self.Yaw)
        self.Y += Vx * self.dt * np.sin(self.Yaw)

# Controller : Pure pursuit
class PurePursuit(object):
    def __init__(self, L=4.0, lookahead_time=1.0):
        """
        Pure Pursuit path tracking controller.
        L: wheelbase length
        lookahead_time: time to compute lookahead distance
        """
        self.L = L
        self.t_lh = lookahead_time
        self.epsilon = 1e-6

    def ControllerInput(self, coeff, Vx):
        """
        Compute steering command based on polynomial path and speed.
        coeff: path polynomial coefficients
        Vx: longitudinal speed
        Updates self.u with steering angle.
        """
        # compute lookahead distance
        d_lh = Vx * self.t_lh
        # lateral offset at lookahead distance
        y_lh = Polyval(coeff, d_lh)
        # steering law
        self.u = np.arctan2(2 * self.L * y_lh, d_lh**2 + y_lh**2 + self.epsilon)

# Main simulation
def main():
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0
    X_lane = np.arange(0.0, 100.0, 0.1)
    Y_lane_L, Y_lane_R = lane(X_lane)

    ego_vehicle = VehicleModel_Lat(step_time, Vx)
    controller = PurePursuit()

    time = []
    X_ego = []
    Y_ego = []

    for i in range(int(simulation_time / step_time)):
        time.append(step_time * i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)

        # generate reference lanes ahead
        X_ref = np.arange(ego_vehicle.X, ego_vehicle.X + 5.0, 1.0)
        Y_ref_L, Y_ref_R = lane(X_ref)
        global_points_L = np.column_stack((X_ref, Y_ref_L)).tolist()
        global_points_R = np.column_stack((X_ref, Y_ref_R)).tolist()

        # transform to local frame
        local_points_L = Global2Local(global_points_L, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        local_points_R = Global2Local(global_points_R, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)

        # fit polynomials
        coeff_L = Polyfit(local_points_L, num_order=3)
        coeff_R = Polyfit(local_points_R, num_order=3)
        # center path
        coeff_path = BothLane2Path(coeff_L, coeff_R)

        # controller computes steering
        controller.ControllerInput(coeff_path, Vx)
        ego_vehicle.update(controller.u, Vx)

    # plotting
    plt.figure(figsize=(13, 2))
    plt.plot(X_lane, Y_lane_L, 'k--')
    plt.plot(X_lane, Y_lane_R, 'k--', label="Reference")
    plt.plot(X_ego, Y_ego, 'b-', label="Vehicle Position")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()