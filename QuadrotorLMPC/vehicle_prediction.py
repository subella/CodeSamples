import numpy as np
from numpy.polynomial import polynomial
from scipy.spatial.transform import Rotation
import cvxopt
from math import factorial
import matplotlib.pyplot as plt


def predict_next_state_continuous(A, B, x, u):
    dt = self.root.dt
    x_dot = np.matmul(A, x) + np.matmul(B, u)
    return x + x_dot * dt

def predict_next_state_discrete(A, B, x, u):
    return A.dot(x) + B.dot(u)

def predict_next_state(f, state, dt):
    next_state = state.copy()
    x_dot = f(state.x, state.u)
    next_state.x += x_dot * dt
    next_state.timestamp += dt
    return next_state

#TODO figure where to put this
def get_full_x(state):
    # x = [x,y,z,x',y',z',th,phi,psi,th',phi',psi']

    px = state.x[0]
    py = state.x[1]
    pz = state.x[2]
    psi = state.x[3]
    v = state.x[4]

    u1 = state.u[0]

    x = np.zeros((12,1))
    x[0] = px
    x[1] = py
    x[2] = pz
    x[3] = v * np.cos(psi)
    x[4] = v * np.sin(psi)
    x[5] = 0
    x[6] = 0
    x[7] = 0
    x[8] = psi
    x[9] = 0
    x[10] = 0
    x[11] = u1
    return x



class Vehicle_State():
    '''
        Class to hold the state of the vehicle
        ...
        Attributes
        ----------
        x : 5, np array
            Current state vector ([px,py,pz,theta,v])
        u : 2, np array
            Current control input ([u1, u2])
        timestamp: float
            Current time
        '''
    def __init__(self, x=np.zeros(5), u=np.zeros(2), timestamp=0.):
        self.x = x
        self.u = u
        self.timestamp = timestamp

    def copy(self):
        return Vehicle_State(self.x.copy(), self.u.copy(), self.timestamp)

class Base_Vehicle_Class(object):
    def __init__(self, initial_state):
        self.state = initial_state.copy()

    def get_state(self):
        return self.state.copy()

process_cov =1e-7
sensor_cov = 1e-4
class Car(Base_Vehicle_Class):
    """
        Vehicle Controller and Dynamics class
        ...
        Attributes
        ----------
        state: CarState
            Class to hold x, u and timestamp
        noisy_state: CarState
            ground truth state augmented with gaussian noise
    """
    def __init__(self, initial_state):
        super(Car, self).__init__(initial_state)
        self.noisy_state = initial_state.copy()
        self.Q = process_cov*np.eye(5)
        self.Q[2,2] = 0
        self.R = sensor_cov*np.eye(5)
        self.R[2,2] = 0
        # self.noisy_state.x = self.get_noise_state()

    @staticmethod
    def f(x, u):
        """
        Nonlinear dynamics model for vehicle dynamics
        Computes x_dot given x and u
        RETURNS CONTINUOUS DYNAMICS
        ...
        Parameters
        ----------
        x : 5, np array
            Current state vector ([px,py,pz,theta,v])
        u : 2, np array
            Current control input ([u1, u2])
        Returns
        -------
        x_dot : 5x1 np array
            Derivative of current state
        """
        px = x[0]
        py = x[1]
        pz = x[2]
        theta = x[3]
        v = x[4]
        u1 = u[0]
        u2 = u[1]

        px_dot = v * np.cos(theta)
        py_dot = v * np.sin(theta)
        pz_dot = 0
        theta_dot = u1
        v_dot = u2

        x_dot = np.array([px_dot, py_dot, pz_dot, theta_dot, v_dot])
        return x_dot

    @staticmethod
    def J(x, u, dt):
        """
        Jacobian of discretized f(x,u)
        | px[k]   |   | px[k-1] + v * cos(theta)*dt |
        | py[k]   |   | py[k-1] + v * sin(theta)*dt |
        | pz[k]   | = | pz[k-1] + 0*dt              |
        | theta[k]|   | theta[k-1] + u1*dt          |
        | v[k]    |   | v[k-1] + u2*dt              |
        ...
        Parameters
        ----------
        x : 5, np array
            Current state vector ([px,py,pz,theta,v])
        u : 2, np array
            Current control input ([u1, u2])
        Returns
        -------
        A : 5x5 np array
            Jacobian of f wrt x
            |df1/dx1 ... df1/dxn|
            |...     ... ...    |
            |dfm/dx1 ... dfm/dxn|
        B : 5x2 np array
            Jacobian of f wrt u
        """
        px = x[0]
        py = x[1]
        theta = x[2]
        v = x[3]
        u1 = u[0]
        u2 = u[1]

        A = np.array([[1, 0, 0, -v*np.sin(theta)*dt, np.cos(theta)*dt],
                      [0, 1, 0,  v*np.cos(theta)*dt, np.sin(theta)*dt],
                      [0, 0, 1,                0,             0],
                      [0, 0, 0,                1,             0],
                      [0, 0, 0,                0,             1]
                        ])

        B = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [dt, 0],
                      [0, dt]])

        return A, B

    def add_noise(self, state, cov):
        """
        Adds gaussian noise to state
        Parameters
        ----------
        state: CarState
            State to be augmented with noise
        """
        mean = np.zeros((cov.shape[0]))
        noise = np.random.multivariate_normal(mean, cov)
        state.x += noise
        return state

    def get_noisy_state(self):
        return self.noisy_state.copy()

    def control_fn1(self, time):
        if 0<= time <1:
            self.state.u = np.array([.00004, .0001])
        elif 1<= time < 3:
            self.state.u = np.array([-.0007, -.0001])
        elif 3 <= time < 6:
            self.state.u = np.array([0.0005, .0002])
        elif 6<= time < 8:
            self.state.u = np.array([-0.0005, .0002])

    def control_fn2(self, time):
        if 0<= time <1:
            self.state.u = np.array([.08, .002])
        elif 1<= time < 3:
            self.state.u = np.array([.08, .002])
        elif 3 <= time < 6:
            self.state.u = np.array([-0.02, .001])
        elif 6<= time < 8:
            self.state.u = np.array([-0.02, .002])
    
    def control_fn3(self, time):
        if 0<= time <1:
            self.state.u = np.array([.2, .03])
        elif 1<= time < 3:
            self.state.u = np.array([-.2, -.04])
        elif 3 <= time < 6:
            self.state.u = np.array([.1, .05])
        elif 6<= time < 8:
            self.state.u = np.array([-.1, .02])

    def update(self, timestamp):
        """
        Called every step, update state
        Parameters
        ----------
        timestamp : float
            Current time        
        """
        dt = timestamp - self.state.timestamp
        x_dot = self.f(self.state.x, self.state.u)

        #TODO make this less confusing
        self.state.x += x_dot * dt
        self.state = self.add_noise(self.state.copy(), cov=self.Q)
        # u is never updated (constant v, theta)
        self.state.u = self.state.u
        # self.control_fn3(timestamp)
        self.state.timestamp = timestamp
        self.noisy_state = self.add_noise(self.state.copy(), cov=self.R)


class EKF_Estimator(Base_Vehicle_Class):
    """
    Extended Kalman filter for estimating car state
    ...
    Attributes
    ----------
    state: CarState
        Estimated state of target vehicle
    prev_A: 5x5 np.array
        State transition matrix for previous timestep
    prev_P: 5x5 np.array
        State covariance estimate for previous timestep
    Q: 5x5 np.array
        State model noise covariance matrix
    R: 5x5 np.array
        Sensor measuremnet noise covariance matrix
    H : 5x5 np.array
        Measurement matrix
    f: function (f(x,u))
        Nonlinear dynamics model of target vehicle
    J: function (f(x,u,dt))
        Nonlinear DISCRETE jacobian
    w : 5x1 np.array
        Sensor noise
    v : 5x1 np.array
        Model noise
    """
    def __init__(self, f, J, target_initial_state):
        super(EKF_Estimator, self).__init__(target_initial_state)
        # initialize to identity, but is overwritten on first loop
        self.prev_A = np.eye(5)
        self.prev_P = np.eye(5)

        # Assumed to be identity
        self.Q = process_cov*np.eye(5)
        self.Q[2,2] = 0
        self.R = .75*sensor_cov*np.eye(4)
        self.R[2,2] = 0
        self.H = np.array([[1,0,0,0,0],
                           [0,1,0,0,0],
                           [0,0,1,0,0],
                           [0,0,0,1,0]])

        self.f = f
        self.J = J

        self.w = .1*np.array([1,1,1,1,1])
        self.v = .1*np.array([1,1,1,1,1])

    def prediction(self, dt):
        """
        Prediction step of ekf
        Computes x_dot for current observed state
        Computes Jacobian for correction step
        """
        x_dot = self.f(self.state.x, self.state.u)
        self.prev_A, _ = self.J(self.state.x, self.state.u, dt)
        self.state.x += x_dot * dt

    def correction(self, observed_state, dt):
        observation = observed_state.x[:4]
        P = self.prev_A.dot(self.prev_P).dot(self.prev_A.T) + self.Q
        y = observation - (self.H.dot(self.state.x))
        S = self.H.dot(P).dot(self.H.T) + self.R
        K = P.dot(self.H.T).dot(np.linalg.pinv(S))
        self.state.x += K.dot(y)
        self.prev_P = P - (K.dot(self.H).dot(P))

    def update(self, timestamp, observed_state=None):
        """
        Called every simulation step
        If observed_state is given, we can perform
        corrective step
        """
        dt = timestamp - self.state.timestamp
        self.state.timestamp = timestamp
        # self.state.u = observed_state.u if observed_state else self.state.u
        self.prediction(dt)
        if observed_state is not None:
            self.correction(observed_state, dt)

    def predict_next_state(self, f, state, dt):
        next_state = state.copy()
        x_dot = f(state.x, state.u)
        next_state.x += x_dot * dt
        next_state.timestamp += dt
        return next_state

class Regression_Estimator():
    """
        Estimates target trajectory through least squares
        and an acceleration regulator
        http://uav.ust.hk/wordpress/wp-content/uploads/2016/09/IROS2016-jing.pdf
        ...
        Attributes
        ----------
        previous_states : list
            Contains all observed target states for 
            the duration of the sliding window
        L : float
            Length of the sliding window
        m : float
            Length of the predicition window
        poly_order : float
            Degree of fitted polynomial
            TODO: NOTE THIS IS OFF BY ONE
            OF ACTUAL DEFINITION OF 
            POLYNOMIAL ORDER
        lambda_t : float
            Regularization weighting gain

        NOTE: THIS IS ONLY CAPABLE OF ESTIMATING X,Y,Z AND
        DERIVATIVES, NOT THETA
    """
    def __init__(self, target_initial_state, L=100, m=1, poly_order=6, 
                 lambda_t=0.00025):
        self.previous_states = [target_initial_state]
        self.polynomials = None
        self.state = target_initial_state

        self.L = L
        self.m = m
        self.poly_order = poly_order
        self.lambda_t = lambda_t


    def get_state(self, timestamp=None):
        """
        Creates a CarState object and fills it based
        on the polynomial's estimate at time = timestamp
        """
        if self.trajectory_position(timestamp) is None:
            return None
        pos, vel,_,_ = self.trajectory_position(timestamp)
        state = Vehicle_State()
        state.x = np.array([pos[0], pos[1], pos[2], 
                            pos[3], np.linalg.norm((vel[0], vel[1]))])
        state.timestamp = timestamp
        return state.copy()
        
    def predict_next_state(self, f, state, dt):
        time = state.timestamp + dt
        if self.get_state(time) is None:
            return predict_next_state(f, state, dt)
        else:
            return self.get_state(time)

    def update(self, timestamp, observed_state=None):
        """
        Called every sim step.
        Fills sliding window and solves the 
        polnomial fitting QP problem
        """
        
        # If no new state passed, we have nothing to 
        # update so return
        if observed_state is None:
            return
        
        # Collect states until sliding window is filled
        self.previous_states.append(observed_state)
        if len(self.previous_states) < self.L + 1:
            return
        self.previous_states.pop(0)
        
        # Sliding window start time: t_l (time of first state in sliding window)
        # Current time: t_0 (time of last state in sliding window)
        # End time of predicition horizon: t_m (current time + constant prediction time)
        t_l = self.previous_states[0].timestamp
        t_0 = self.previous_states[-1].timestamp
        t_m = t_0 + self.m

        # use t_l as reference time = 0
        # Using relative times simplifies the problem
        # compared to absolute time
        # t_0 -= t_l
        # t_m -= t_l

        # For each observed state, we create a polynmoial at that the time of the state
        # then concat them all.
        # Size : poly_order x L
        t = self.make_t(self.previous_states, self.poly_order, t_l)

        # Puts the acceleration regualtor (eq 6) in matrix form
        H = self.make_H(self.poly_order, derivative_to_minimize=2, t_l = 0, t_m=t_m - t_l)

        
        # t.dot(t.T) comes form the least squares QP subproblem (eqs 17-20)
        # self.L * self.lambda_t * H comes from the regulation (eqs 21-24)
        P = t.dot(t.T) + self.L * self.lambda_t * H

        # We solve x,y,z qp problems separately and append them to polynomials
        # array. 
        polynomials = []
        for axis in range(4):
            # p is a matrix of all observed states for given axis
            p = self.make_p(self.previous_states, axis)
            T_hat = np.squeeze(np.linalg.inv(P).dot(t).dot(p))
            T_hat = polynomial.polytrim(T_hat)
            poly = polynomial.Polynomial(T_hat)
            polynomials.append(poly)

        self.polynomials = polynomials
        
    def trajectory_position(self, time):
        """
        Returns to position and derivatives of the 
        polynomial at time time.
        """
        if self.polynomials is None:
            return
        # Treat t_l as 0 time so we have to subtract it out
        # from the absolute time
        t_l = self.previous_states[0].timestamp
        time = time - t_l

        polynomials = self.polynomials
        pos = np.array([p(time) for p in polynomials])
        vel = np.array([p.deriv(1)(time) for p in polynomials])
        acc = np.array([p.deriv(2)(time) for p in polynomials])
        jerk = np.array([p.deriv(3)(time) for p in polynomials])
        return pos, vel, acc, jerk

    @staticmethod
    def make_t(states, poly_order, t_l):
        L = len(states)
        t = np.zeros((poly_order, L))
        for row in range(poly_order):
            for col in range(L):
                t[row][col] = (states[col].timestamp - t_l)**row
        return t

    @staticmethod
    def make_p(states, axis):
        return np.array([[state.x[axis] for state in states]]).T

    @staticmethod
    def make_H(poly_order, derivative_to_minimize, t_l, t_m):

        def a(n, derivative_to_minimize=2):
            return factorial(n)/factorial(n - derivative_to_minimize)

        H = np.zeros((poly_order, poly_order))
        for row in range(derivative_to_minimize, poly_order):
            for col in range(derivative_to_minimize, poly_order):
                integral_exponent = row + col - 2 * derivative_to_minimize + 1
                H[row, col] = a(row)*a(col)/integral_exponent * (t_m**(integral_exponent) - t_l**(integral_exponent))

        return H

def simulate(car, reg=None, ekf=None, control_fn=None, dt=0.01, duration=10.,
             sensor_hz = 40., use_noise=True, animate=False):
    time = 0
    last_update = 0
    ground_truth = []
    noisy_ground_truth = []
    reg_estimate = [] if reg else None
    ekf_estimate = [] if ekf else None
    reg_key_times = [] if reg else None
    reg_poly = []
    observed_state_array = []
    while time < duration:
        if control_fn is not None:
            car.state.u = control_fn(time)

        observed_state = None
        if 1/(time - last_update + 1e-6) <= sensor_hz:
            last_update = time
            if use_noise:
                observed_state = car.get_noisy_state()
            else:
                observed_state = car.get_state()
            observed_state_array.append(observed_state)
                
        if ekf:
            ekf.update(time, observed_state)
            ekf_estimate.append(ekf.get_state())
            
        if reg:
            # reg.update(time, observed_state)
            reg.update(time, ekf.get_state())

            if reg.get_state(time):
                reg_estimate.append(reg.get_state(time))
                if observed_state is not None:
                    reg_key_times.append(reg.get_state(time))
            if animate:
                if reg.get_state(time):
                    reg_poly = []
                    for t in range(int(100*(time-(1/dt)/100.*reg.L*dt)), int(100*(time+1.5)), int(100*dt)):
                        reg_poly.append(reg.get_state((t/100.)))
                        # print reg.get_state(t)
        ground_truth.append(car.get_state())
        noisy_ground_truth.append(car.get_noisy_state())

        if animate:
            plot_comparison(ground_truth, noisy_ground_truth=noisy_ground_truth, ekf_estimate=ekf_estimate,
                            reg_estimate=reg_estimate, reg_key_times=None, reg_poly=reg_poly, observed_states=None)
            plt.pause(0.0000001)


        car.update(time)
        time += dt

    return ground_truth, noisy_ground_truth, reg_estimate, ekf_estimate, reg_key_times

def plot_car_trajectory(car_state_array):
    X = car_state_array[:,0]
    Y = car_state_array[:,1]
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title("Vehicle Trajectory")
    plt.xlabel("X Position [m]")
    plt.ylabel("Y Position [m]")
    plt.plot(X, Y)

def plot_comparison(ground_truth, noisy_ground_truth=None,
                    reg_estimate=None, reg_key_times=None, 
                    reg_poly=None, ekf_estimate=None,
                    observed_states=None):
    plt.gca().set_aspect('equal', adjustable='box')
    plt.clf()
    # plt.rcParams["figure.figsize"] = (20,20)

    ground_truth_X = [state.x[0] for state in ground_truth]
    ground_truth_Y = [state.x[1] for state in ground_truth]
    plt.plot(ground_truth_X, ground_truth_Y, 'g', label="Ground Truth")

    if noisy_ground_truth is not None:
        noisy_ground_truth_X = [state.x[0] for state in noisy_ground_truth]
        noisy_ground_truth_Y = [state.x[1] for state in noisy_ground_truth]
        plt.scatter(noisy_ground_truth_X, noisy_ground_truth_Y, s=0.8, c="gray", label="Noisy Ground Truth")
   
    if reg_estimate is not None:
        reg_estimate_X = [state.x[0] for state in reg_estimate]
        reg_estimate_Y = [state.x[1] for state in reg_estimate]
        plt.plot(reg_estimate_X, reg_estimate_Y, 'b', label="Regression")

    if reg_key_times is not None:
        reg_key_X = [state.x[0] for state in reg_key_times]
        reg_key_Y = [state.x[1] for state in reg_key_times]
        plt.scatter(reg_key_X, reg_key_Y, c='black', label="Regression Key Times")

    if reg_poly is not None:
        reg_poly_X = [state.x[0] for state in reg_poly]
        reg_poly_Y = [state.x[1] for state in reg_poly]
        plt.plot(reg_poly_X, reg_poly_Y, c='red', label="Regression Polynomial")

    if observed_states is not None:
        observed_state_X = [state.x[0] for state in observed_states]
        observed_state_Y = [state.x[1] for state in observed_states]
        plt.scatter(observed_state_X, observed_state_Y, c='pink', label="Observed States")

    if ekf_estimate is not None:
        ekf_estimate_X = [state.x[0] for state in ekf_estimate]
        ekf_estimate_Y = [state.x[1] for state in ekf_estimate]
        plt.plot(ekf_estimate_X, ekf_estimate_Y, 'r', label="EKF")

    plt.legend(loc='upper left')
    # plt.show()

if __name__ == "__main__":
    # # Circular Trajectory
    car_initial_state = Vehicle_State(np.array([0., 0., 0., 0, 1]), np.array([1,0.]))

    # reg_initial_state = Vehicle_State(car_initial_state.x[:4])
    car = Car(car_initial_state)
    ekf = EKF_Estimator(car.f, car.J, car.get_noisy_state())
    reg = Regression_Estimator(car.get_noisy_state())
    ground_truth, noisy_ground_truth, reg_estimate,\
     ekf_estimate, reg_key_times = simulate(car, reg=reg, ekf=ekf, use_noise=True, animate=False)


    plot_comparison(ground_truth, noisy_ground_truth=noisy_ground_truth, reg_estimate=reg_estimate,
                    reg_key_times=None, ekf_estimate=ekf_estimate)
    plt.show()

    # Straight Line
    car_initial_state = Vehicle_State(np.array([0., 0., 0., np.pi/4, 1]), np.array([0.,0.]))
    car_initial_state =  Vehicle_State(np.array([0.25, -0.5, 0.45, 0, 1]), np.array([0.0,0.]))
    car = Car(car_initial_state)
    ekf = EKF_Estimator(car.f, car.J, car.get_state())
    reg = Regression_Estimator(car.get_state())

    ground_truth, noisy_ground_truth, reg_estimate, ekf_estimate, _ = simulate(car, reg=reg, ekf=ekf,
                                                                                 animate=False)
    plot_comparison(ground_truth, noisy_ground_truth=noisy_ground_truth, 
                    reg_estimate=reg_estimate, ekf_estimate=ekf_estimate)
    plt.show()
    
    # # Sinusoidal Trajectory
    # car_initial_state = Vehicle_State(np.array([0., 0., 0., 0, 1]), np.array([0.,0.]))
    # # reg_initial_state = RegressionState(car_initial_state.x[:3])
    # car = Car(car_initial_state)
    # ekf = EKF_Estimator(car.f, car.J, car.get_state())
    # reg = Regression_Estimator(car.get_state())
    # def controller(t):
    #     return np.array([2, 2])
    # ground_truth, noisy_ground_truth, reg_estimate, ekf_estimate, _ = simulate(car, reg=reg, ekf=ekf, 
    #                                                                     control_fn=controller, use_noise=True,)
    # plot_comparison(ground_truth, noisy_ground_truth=noisy_ground_truth, reg_estimate=reg_estimate, ekf_estimate=ekf_estimate)
    # plt.show()