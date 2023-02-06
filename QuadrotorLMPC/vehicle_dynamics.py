import numpy as np

class Vehicle_State():
    def __init__(self, x=np.zeros(5), u=np.zeros(2), timestamp=0):
        '''
        x : 5, np array
            Current state vector ([px,py,pz,theta,v])
        u : 2, np array
            Current control input ([u1, u2])
        timestamp: float
            Current time
        '''
        self.x = x
        self.u = u
        self.timestamp = timestamp

    def copy(self):
        return Vehicle_State(self.x.copy(), self.u.copy(), self.timestamp)

def car_dynamics_fn(x, u):
    """
    Nonlinear dynamics model for vehicle dynamics
    Computes x_dot given x and u
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