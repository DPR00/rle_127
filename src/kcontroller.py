import numpy as np
from qpoases import PyQProblemB as QProblemB
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel

class WQPController(object):
    """
    This controller uses the position of the four limbs as default in the order:
    front right leg, front left leg, rear right leg, rear left leg

    """
    def __init__(self, weights, lambdas, dt):
        self.w1 = weights[0]
        self.w2 = weights[1]
        self.w3 = weights[2]
        self.w4 = weights[3]
        self.lambda1 = lambdas[0]
        self.lambda2 = lambdas[1]
        self.lambda3 = lambdas[2]
        self.lambda4 = lambdas[3]
        self.dt = dt
        # Joint limits
        self.qmin = np.array([-1.4, -1.4, -2.5, -1.4, -1.4, -2.5, -1.4, -1.4,
                              -2.5, -1.4, -1.4, -2.5])
        self.qmax = np.array([1.4, 1.4, 2.5, 1.4, 1.4, 2.5, 1.4, 1.4, 2.5, 1.4,
                              1.4, 2.5])
        self.dqmax = 10.0*np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                               1.0, 1.0, 1.0])
        self.dqmin = -self.dqmax
        # Bounds for the floating base
        low = -1e6
        high = 1e6
        self.lfb = np.array([low, low, low, low, low, low, low])
        self.ufb = np.array([high, high, high, high, high, high, high])
        
    def get_dq(self, q, e1, J1, e2, J2, e3, J3, e4, J4):
        de1 = self.lambda1*e1
        de2 = self.lambda2*e2
        de3 = self.lambda3*e3
        de4 = self.lambda4*e4

        W = self.w1*np.dot(J1.T, J1) + self.w2*np.dot(J2.T, J2) + self.w3*np.dot(J3.T, J3) + self.w4*np.dot(J4.T, J4)
        p = -2*(self.w1*np.dot(J1.T, de1) + self.w2*np.dot(J2.T, de2) + self.w3*np.dot(J3.T, de3) + self.w4*np.dot(J4.T, de4) )

        lower_limits = np.maximum((self.qmin-q[7:])/self.dt, self.dqmin)
        upper_limits = np.minimum((self.qmax-q[7:])/self.dt, self.dqmax)

        lower_limits = np.hstack((self.lfb, lower_limits))
        upper_limits = np.hstack((self.ufb, upper_limits))

        # Solver
        solver = QProblemB(19)
        options = Options()
        options.setToMPC()
        options.printLevel = PrintLevel.LOW
        solver.setOptions(options)

        nWSR = np.array([10])
        solver.init(W, p, lower_limits, upper_limits, nWSR)
        dq = np.zeros(19)
        solver.getPrimalSolution(dq)
        return dq


class WQPControllerBase(object):
    """
    This controller uses the position of the four limbs as default in the order:
    front right leg, front left leg, rear right leg, rear left leg

    """
    def __init__(self, weights, lambdas, dt):
        self.w1 = weights[0]
        self.w2 = weights[1]
        self.w3 = weights[2]
        self.w4 = weights[3]
        self.w5 = weights[4]
        self.lambda1 = lambdas[0]
        self.lambda2 = lambdas[1]
        self.lambda3 = lambdas[2]
        self.lambda4 = lambdas[3]
        self.lambda5 = lambdas[4]
        self.dt = dt
        # Joint limits
        self.qmin = np.array([-1.4, -1.4, -2.5, -1.4, -1.4, -2.5, -1.4, -1.4,
                              -2.5, -1.4, -1.4, -2.5])
        self.qmax = np.array([1.4, 1.4, 2.5, 1.4, 1.4, 2.5, 1.4, 1.4, 2.5, 1.4,
                              1.4, 2.5])
        self.dqmax = 10.0*np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                               1.0, 1.0, 1.0])
        self.dqmin = -self.dqmax
        # Bounds for the floating base
        low = -1e6
        high = 1e6
        self.lfb = np.array([low, low, low, low, low, low, low])
        self.ufb = np.array([high, high, high, high, high, high, high])
        
    def get_dq(self, q, e1, J1, e2, J2, e3, J3, e4, J4, e5, J5):
        de1 = self.lambda1*e1
        de2 = self.lambda2*e2
        de3 = self.lambda3*e3
        de4 = self.lambda4*e4
        de5 = self.lambda5*e5

        # print J5.shape
        # print de5.shape
        
        W = self.w1*np.dot(J1.T, J1) + self.w2*np.dot(J2.T, J2) + self.w3*np.dot(J3.T, J3) + self.w4*np.dot(J4.T, J4) + self.w5*np.dot(J5.T, J5)
        p = -2*(self.w1*np.dot(J1.T, de1) + self.w2*np.dot(J2.T, de2) + self.w3*np.dot(J3.T, de3) + self.w4*np.dot(J4.T, de4) + self.w5*np.dot(J5.T, de5))

        lower_limits = np.maximum((self.qmin-q[7:])/self.dt, self.dqmin)
        upper_limits = np.minimum((self.qmax-q[7:])/self.dt, self.dqmax)

        lower_limits = np.hstack((self.lfb, lower_limits))
        upper_limits = np.hstack((self.ufb, upper_limits))

        # Solver
        solver = QProblemB(19)
        options = Options()
        options.setToMPC()
        options.printLevel = PrintLevel.LOW
        solver.setOptions(options)

        nWSR = np.array([10])
        solver.init(W, p, lower_limits, upper_limits, nWSR)
        dq = np.zeros(19)
        solver.getPrimalSolution(dq)
        return dq
