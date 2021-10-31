import numpy as np


def skew(v):
    S = np.array([[0., -v[2], v[1]],
                  [v[2], 0., -v[0]],
                  [-v[1], v[0], 0.]])
    return S


def rotation(omega, th):
    """
    Generate a rotation matrix from the angular velocity and angle

    """
    skomega = skew(omega)
    R = np.eye(3) + np.sin(th)*skomega + (1-np.cos(th))*skomega.dot(skomega)
    return R



def transform(omega, v, th):
    """
    Generate a homogenenous transformation from a screw and an angle

    """
    T = np.eye(4)
    skomega = skew(omega)
    p = th*np.eye(3) + (1-np.cos(th))*skomega + \
        (th-np.sin(th))*skomega.dot(skomega)
    p = np.dot(p, v)
    R = rotation(omega, th)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T

def Trotx(angle):
    """
    Homogeneous transformation matrix from a rotation about x

    """
    T = np.eye(4)
    ca = np.cos(angle); sa = np.sin(angle)
    R = np.array([[1., 0., 0.],
                  [0., ca, -sa],
                  [0., sa, ca]])
    T[0:3,0:3] = R
    return T


def Troty(angle):
    """
    Homogeneous transformation matrix from a rotation about y

    """
    T = np.eye(4)
    ca = np.cos(angle); sa = np.sin(angle)
    R = np.array([[ ca, 0., sa],
                  [ 0., 1., 0.],
                  [-sa, 0., ca]])
    T[0:3,0:3] = R
    return T


def Trotz(angle):
    """
    Homogeneous transformation matrix from a rotation about z

    """
    T = np.eye(4)
    ca = np.cos(angle); sa = np.sin(angle)
    R = np.array([[ ca, -sa, 0.],
                  [ sa,  ca, 0.],
                  [ 0.,  0., 1.]])
    T[0:3,0:3] = R
    return T

def Ttransl(d):
    """
    Homogeneous transformation matrix for a translation d

    """
    T = np.eye(4)
    T[0,3] = d[0]
    T[1,3] = d[1]
    T[2,3] = d[2]
    return T

def rotationFromQuat(q):
    """
    Rotation Matrix from a quaternion

    """
    R = np.zeros([3,3])
    ew = q[0]; ex = q[1]; ey = q[2]; ez = q[3]
    R[0,0] = 2.0*(ew*ew + ex*ex)-1.0
    R[0,1] = 2.0*(ex*ey - ew*ez)
    R[0,2] = 2.0*(ex*ez + ew*ey)
    R[1,0] = 2.0*(ex*ey + ew*ez)
    R[1,1] = 2.0*(ew*ew + ey*ey)-1.0
    R[1,2] = 2.0*(ey*ez - ew*ex)
    R[2,0] = 2.0*(ex*ez - ew*ey)
    R[2,1] = 2.0*(ey*ez + ew*ex)
    R[2,2] = 2.0*(ew*ew + ez*ez)-1.0
    return R

def Tmat(q):
    """
    Matrix that relates the angular velocity and the rate of change of the
    quaternion as: w = Tmat*dQ.

    The input is the pose of the base

    """
    T = np.array([
        [-2.0*q[4],  2.0*q[3], -2.0*q[6],  2.0*q[5]],
        [-2.0*q[5],  2.0*q[6],  2.0*q[3], -2.0*q[4]],
        [-2.0*q[6], -2.0*q[5],  2.0*q[4],  2.0*q[3]]])
    return T

def quaternionMult(q1,q2):
    
    qout = np.zeros(4)
    qout[0] = -q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3] + q1[0]*q2[0]
    qout[1] =  q1[0]*q2[1] - q1[3]*q2[2] + q1[2]*q2[3] + q1[1]*q2[0]
    qout[2] =  q1[3]*q2[1] + q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0]
    qout[3] = -q1[2]*q2[1] + q1[1]*q2[2] + q1[0]*q2[3] + q1[3]*q2[0]
    return qout

