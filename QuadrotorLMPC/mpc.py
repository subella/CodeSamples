import numpy as np
import scipy
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import collections as mc
from matplotlib import colors as mcolors
import itertools as it
import cvxopt
import sys 
import seaborn as sns
sns.set()
sns.set_style("whitegrid")
from vehicle_prediction import *

def nonlinear_quadrotor_dynamics(x, u, m, g, J):
    x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, phi_dot, theta_dot, psi_dot = x.reshape(-1).tolist()
    ft, tau_x, tau_y, tau_z = u.reshape(-1).tolist()

    Ix = J[0,0]
    Iy = J[1,1]
    Iz = J[2,2]
    x_dot = np.array([
                      x_dot,
                      y_dot,
                      z_dot,
                      -ft/m*(np.sin(phi)*np.sin(psi)+np.cos(phi)*np.cos(psi)*np.sin(theta)),
                      -ft/m*(np.cos(phi)*np.sin(psi)*np.sin(theta)-np.cos(psi)*np.sin(phi)),
                      g-ft/m*np.cos(phi)*np.cos(theta),
                      phi_dot,
                      theta_dot,
                      psi_dot,
                      (Iy-Iz)/Ix*theta_dot*psi_dot+tau_x/Ix,
                      (Iz-Ix)/Iy*phi_dot*psi_dot+tau_y/Iy,
                      (Ix-Iy)/Iz*phi_dot*theta_dot+tau_z/Iz])
    return x_dot

def cl_nonlinear(x, u, m, g, J):
    x = np.array(x)
    dot_x = nonlinear_quadrotor_dynamics(x, u + np.array([m * g, 0, 0, 0]), m, g, J)
    return dot_x


class MPC_Solver(Base_Vehicle_Class):
    def __init__(self, initial_state, target_state_estimator, target_state_predictor, m, g, J):
        super(MPC_Solver, self).__init__(initial_state)
        self.state_estimator = target_state_estimator
        self.state_predictor = target_state_predictor
        self.m = m
        self.g = g
        self.J = J

    def linearize_quadrotor_dynamics_discrete(self, dt):
        """
        Returns linearized matrices A,B so that
        x[k+1] = Ax[k] + Bu[k]

        x = [x,y,z,x',y',z',phi,theta,psi,phi',theta',psi']
        """
        #TODO double check coordinate frame
        
        Jx = self.J[0,0]
        Jy = self.J[1,1]
        Jz = self.J[2,2]
        A = np.zeros((12,12))
        A[0,3] = 1
        A[1,4] = 1
        A[2,5] =1 
        A[3,7] = -self.g
        A[4,6] = self.g
        A[6,9] = 1
        A[7,10] = 1
        A[8,11] = 1

        B = np.zeros((12,4))
        B[5,0] = -1/self.m
        B[9, 1] = 1/Jx
        B[10, 2] = 1/Jy
        B[11, 3] = 1/Jz
        A *= dt
        A += np.eye(12)
        B *= dt
        return A, B

    def linearize_quadrotor_dynamics(self):
        """
        Returns linearized matrices A,B about hover

        x = [x,y,z,x',y',z',phi,theta,psi,phi',theta',psi']
        """
        #TODO double check coordinate frame
        
        Jx = self.J[0,0]
        Jy = self.J[1,1]
        Jz = self.J[2,2]

        A = np.zeros((12,12))
        A[0,3] = 1
        A[1,4] = 1
        A[2,5] =1 
        A[3,7] = -self.g
        A[4,6] = self.g
        A[6,9] = 1
        A[7,10] = 1
        A[8,11] = 1

        B = np.zeros((12,4))
        B[5,0] = -1/self.m
        B[9, 1] = 1/Jx
        B[10, 2] = 1/Jy
        B[11, 3] = 1/Jz
        return A, B

    def make_r(self, car_state, P, C, dt):
        r = np.zeros((P, 12, 1))
        Cr = np.zeros((P, 8, 1))
        next_state = car_state
        self.next_states = []

        for i in range(P): 
            #TODO multiple use of f, clean this up
            next_state = predict_next_state(self.state_estimator.f, next_state, dt)
            self.next_states.append(next_state)
            r[i] = get_full_x(next_state)
            Cr[i] = C.dot(get_full_x(next_state))
        return r, Cr

    def make_A(self, P, C, dt):
        Ap = np.zeros((P, 12, 12))
        CAp = np.zeros((P, 8, 12))
        A, _ = self.linearize_quadrotor_dynamics_discrete(dt)
        A_power = A
        for i in range(P):
            Ap[i] = A_power
            CAp[i] = C.dot(A_power)
            A_power = np.matmul(A, A_power)  
        return Ap, CAp

    def make_B(self, P, M, C, dt):
        Bp = np.zeros((P, M, 12, 4))
        CBp = np.zeros((P, M, 8, 4))
        A, B = self.linearize_quadrotor_dynamics_discrete(dt)
        for col in range(M):
            for row in range(col, P):
                Bp[row,col] =  np.matmul(np.linalg.matrix_power(A, (row - col)), B)
                CBp[row,col] =  C.dot(np.matmul(np.linalg.matrix_power(A, (row - col)), B))

        return Bp, CBp

    def calc_u(self, car_state, return_home):
        P = 12
        M = 5
        S = self.state.x.shape[0]
        dt = 0.1
        C = np.zeros((8,12))
        C[0,0] = 1
        C[1,1] = 1
        C[2,2] = 1
        C[3,3] = 1
        C[4,4] = 1
        C[5,5] = 1
        C[6,8] = 1
        C[7,11] = 1
        r, Cr = self.make_r(car_state, P=P, C=C, dt=dt)
        self.r = r
        rp = Cr

        Ap, CAp = self.make_A(P=P, C=C, dt=dt)
        Bp, CBp = self.make_B(P=P, M=M, C=C, dt=dt)
        
        #TODO lots of hardcoding here
        rp = rp.reshape((8*P, 1))
        Ap = Ap.reshape((12*P, 12))
        Bp = Bp.swapaxes(2,1).reshape(Bp.shape[0] * Bp.shape[2],-1)
        CAp = CAp.reshape((8*P, 12))
        CBp = CBp.swapaxes(2,1).reshape(CBp.shape[0] * CBp.shape[2],-1)

        Q = np.zeros((8*P,8*P))

        #TODO so gross
        for i in range(0, 7*8, 8):
            Q[i,i] = 10
            Q[i+1,i+1] = 10
            Q[i+2,i+2] = 10
            Q[i+3,i+3] = 1
            Q[i+4,i+4] = 1
            Q[i+5,i+5] = 1
            Q[i+6, i+6] = 10
            Q[i+7, i+7] = 1
        for i in range(7*8, 12*8, 8):
            Q[i,i] = 1
            Q[i+1,i+1] = 1
            Q[i+2,i+2] = 1
            Q[i+3,i+3] = 1.5
            Q[i+4,i+4] = 1.5
            Q[i+5,i+5] = 1.5
            Q[i+6, i+6] = 1
            Q[i+7, i+7] = 1.5

        R = np.eye(4*M)
        x_k = self.state.x.reshape((12,1))
        q = 0.5

        # QP formulation
        P_ = q*CBp.T.dot(Q).dot(CBp) + (1-q)*R
        q_ = ((q*x_k.T.dot(CAp.T) - q*rp.T).dot(Q).dot(CBp)).T

        # Constraint on z>floor_height for all states
        # Cz selects all of the z positions
        # Cz*(Ap*xk + Bu) >= 0
        floor_height = 0.476
        cz = np.zeros((S))
        cz[2] = 1
        Cz = np.kron(np.eye(P,dtype=int), cz)
        Gz = Cz.dot(Bp)
        hz = floor_height*np.ones((P,1)) - Cz.dot(Ap).dot(x_k)

        # # Constraint on roll to remain in small angle approx.
        # # Cr selects all roll
        # # abs(Cr*(Ap*xk + Bu)) <= 0.5 (rad)
        max_roll = 0.3
        cr = np.zeros((S))
        cr[6] = 1
        Cr = np.kron(np.eye(P,dtype=int), cr)
        I_abs = np.vstack((np.eye(P), -np.eye(P)))
        Gr = I_abs.dot(Cr.dot(Bp))
        hr = max_roll * np.ones((2*P,1)) - I_abs.dot(Cr).dot(Ap).dot(x_k)

        # # Constraint on pitch to remain in small angle approx.
        # # Cp selects all pitch
        # # abs(Cp*(Ap*xk + Bu)) <= 0.5 (rad)
        max_pitch = 0.3
        cp = np.zeros((S))
        cp[7] = 1
        Cp = np.kron(np.eye(P,dtype=int), cp)
        I_abs = np.vstack((np.eye(P), -np.eye(P)))
        Gp = I_abs.dot(Cp.dot(Bp))
        hp = max_pitch * np.ones((2*P,1)) - I_abs.dot(Cp).dot(Ap).dot(x_k)

        P_ = cvxopt.matrix(P_, tc='d')
        q_ = cvxopt.matrix(q_, tc='d')

        G = np.vstack((Gz, Gr, Gp))
        h = np.vstack((hz, hr, hp))

        G = cvxopt.matrix(G, tc='d')
        h = cvxopt.matrix(h, tc='d')

        cvxopt.solvers.options['show_progress'] = False
        solution = cvxopt.solvers.qp(P_, q=q_, G=G, h=h)
        if solution['status'] != 'optimal':
            raise Exception("Solved qp with solution " + str(solution))


        U = np.squeeze(solution['x'])
        # print "x_k", x_k.reshape((12,))
        self.xp = Ap.dot(x_k) + Bp.dot(U.reshape((4*M,1)))
        return U[:4]

    def update(self, timestamp, observed_state, return_home=False):
        self.state_estimator.update(timestamp, observed_state)
        self.state_predictor.update(timestamp, self.state_estimator.get_state())
        self.state.u = self.calc_u(self.state_estimator.get_state(), return_home)
        self.state.x += cl_nonlinear(self.state.x.copy(), self.state.u.copy(), self.m, self.g, self.J) * .01
        self.state.timestamp = timestamp

    def get_next_states(self):
        return self.next_states

    def get_xp(self):
        return self.xp.reshape((12,12))

    def get_rp(self):
        return self.r.reshape((12,12))

def get_plottable_from_state_array(state_array):
    state_len = state_array[0].x.shape[0]
    ret_arr = [[state.x[i] for state in state_array] for i in range(state_len)]
    times = [state.timestamp for state in state_array]
    return ret_arr, times

def get_plottable_from_numpy_array(np_array):
    state_len = np_array.shape[1]
    ret_arr = [[x[i] for x in np_array] for i in range(state_len)]
    return ret_arr

def plot_mpc_prediction(quadrotor_state_array, car_state_array=None,
                        quadrotor_prediction_array=None, car_prediction_array=None,
                        estimator_array=None, noisy_car_state_array=None):

    ax  = plt.axes(projection='3d')  

    def plot_state(state_array, color, label, scatter=False):
        state_vals, times = get_plottable_from_state_array(state_array)
        X,Y,Z = state_vals[0], state_vals[1], state_vals[2]
        if not scatter:
            ax.plot(X, Y, Z, color=color, label=label)
            ax.plot([X[-1]], [Y[-1]], [Z[-1]], color=color, marker='o', markersize=5.)
        else:
            ax.scatter(X, Y, Z, s=0.8, c=color, label=label)

    def plot_x(x_array, color, label):
        state_vals = get_plottable_from_numpy_array(x_array)
        X,Y,Z = state_vals[0], state_vals[1], state_vals[2]
        ax.plot(X, Y, Z, color=color, label=label)
        ax.plot([X[-1]], [Y[-1]], [Z[-1]], color=color, marker='o', markersize=5.)

    plot_state(quadrotor_state_array, color='r', label="Quadrotor Ground Truth")
    if car_state_array is not None:
        plot_state(car_state_array, color='g', label="Car Ground Truth")
    if estimator_array is not None:
        plot_state(estimator_array, color='pink', label="Estimated Car State")

    if noisy_car_state_array is not None:
        plot_state(noisy_car_state_array, color='gray', label="Observed Car State", scatter=True)

    if quadrotor_prediction_array is not None:
        plot_x(quadrotor_prediction_array, color='b', label="Predicted Quadrotor State")

    if car_prediction_array is not None:
        plot_x(car_prediction_array, color='pink', label="Predicted Car State")

    
    # plt.title("MPC Tracking Response")
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.invert_zaxis()
    plt.legend(loc='best', shadow=True, fontsize='small')

def plot_mpc_states(quadrotor_state_array, car_state_array=None, estimator_array=None, noisy_car_state_array=None):
    plot_rows = 4
    plot_cols = 3
    fig, axs = plt.subplots(plot_rows, plot_cols)
    quad_state_vals, times = get_plottable_from_state_array(quadrotor_state_array)
    if car_state_array is not None:
        car_state_vals, car_state_times = get_plottable_from_state_array(car_state_array)
    if estimator_array is not None:
        estimator_state_vals, estimator_state_times = get_plottable_from_state_array(estimator_array)
    if noisy_car_state_array is not None:
        noisy_car_state_vals, noisy_car_state_times = get_plottable_from_state_array(noisy_car_state_array)

    ax_titles = [['X','Y','Z'],
                 ['X_dot','Y_dot','Z_dot'],
                 ['Phi','Theta','Psi'],
                 ['Phi_dot','Theta_dot','Psi_dot']]
    ax_xlabel = "Time [sec]"
    for row in range(plot_rows):
        for col in range(plot_cols):
            axs[row][col].plot(times, quad_state_vals[col + plot_cols*row])
            if car_state_array is not None:
                axs[row][col].plot(car_state_times, car_state_vals[col + plot_cols*row])
            if estimator_array is not None:
                axs[row][col].plot(estimator_state_times, estimator_state_vals[col + plot_cols*row])
            if noisy_car_state_array is not None:
                axs[row][col].plot(noisy_car_state_times, noisy_car_state_vals[col + plot_cols*row])

            axs[row][col].set_title(ax_titles[row][col])
            axs[row][col].set_xlabel(ax_xlabel)

def simulate(mpc_controller, car_controller, dt=0.01, duration=20,  
             sensor_hz = 40., use_noise=True, animate=False):
    time = 0
    last_update = 0
    quadrotor_state_array = []
    car_state_array = []
    noisy_car_state_array = []
    estimator_array = []
    tracking_offset = np.array([0.,0.,0.])
    while time < duration:
        observed_state = None
        if 1/(time - last_update + 1e-6) <= sensor_hz:
            last_update = time
            if use_noise:
                observed_state = car_controller.get_noisy_state()
            else:
                observed_state = car_controller.get_state()
            observed_state.x[:3] += tracking_offset

        mpc_controller.update(time, observed_state)
        car_controller.update(time)

        quadrotor_state_array.append(mpc_controller.get_state())
        #TODO clean this up, prob get rid of full x crap
        full_x = get_full_x(car_controller.get_state()).reshape((12,))
        new_state = Vehicle_State(full_x, car_controller.get_state().u, car_controller.get_state().timestamp)
        car_state_array.append(new_state)

        full_x = get_full_x(mpc_controller.state_estimator.get_state()).reshape((12,))
        new_state = Vehicle_State(full_x, mpc_controller.state_estimator.get_state().u, 
                                  mpc_controller.state_estimator.get_state().timestamp)
        estimator_array.append(new_state)

        full_x = get_full_x(car_controller.get_noisy_state()).reshape((12,))
        new_state = Vehicle_State(full_x, car_controller.get_noisy_state().u, car_controller.get_noisy_state().timestamp)
        noisy_car_state_array.append(new_state)


        time += dt
        if animate:
            plot_mpc_prediction(quadrotor_state_array, car_state_array, 
                        mpc_controller.get_xp(),  mpc_controller.get_rp(),
                        estimator_array=estimator_array,
                        noisy_car_state_array=noisy_car_state_array)
            plt.pause(0.0001)

    return quadrotor_state_array, car_state_array, noisy_car_state_array, estimator_array

if __name__ == "__main__":
    # model parameters
    g = 9.81
    m = 1.
    J = np.eye(3)
    T = 8
    dt = 0.01

    car_initial_state = Vehicle_State(np.array([0.25, -0.5, 0.45, 0., 0.5]), np.array([0.0,0.]))
    quadrotor_initial_state = Vehicle_State(np.array([0.,0.,0.,0,0,0,0,0,0,0,0,0]), np.array([0.,0.,0.,0.]))

    car_controller = Car(car_initial_state)
    ekf_state = car_controller.get_state()
    ekf_state.u = np.array([0.,0.])
    ekf_state.x[2] +=0.0
    ekf_estimator = EKF_Estimator(car_controller.f, car_controller.J, ekf_state)
    reg_predictor = Regression_Estimator(car_controller.get_state())
    mpc_controller = MPC_Solver(quadrotor_initial_state, ekf_estimator, reg_predictor, m, g, J)
    mpc_controller.linearize_quadrotor_dynamics()
    quadrotor_state_array, car_state_array, \
    noisy_car_state_array, estimator_array = simulate(mpc_controller, car_controller,
                                                     animate=False, use_noise=True)

    plot_mpc_prediction(quadrotor_state_array, car_state_array=None, 
                        quadrotor_prediction_array=None, car_prediction_array=None,
                        estimator_array=estimator_array,
                        noisy_car_state_array=noisy_car_state_array)

    plot_mpc_states(quadrotor_state_array, car_state_array=None, 
                    estimator_array=estimator_array,
                    noisy_car_state_array=None)
    plt.show()