import math

import numpy as np



def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    if any(v): #if not all zeros then
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))

    else:
        return np.eye(3) #cross of all zeros only occurs on identical directions


def matrix2quaternion(m):
    b = m
    c = b[0]
    a = b[1]
    d = b[2]
    e = b[3]
    f = b[4]
    g = b[5]
    h = b[6]
    k = b[7]
    b = b[8]
    l = c + f + b
    if 0 < l:
        c = .5 / np.sqrt(l + 1)
        w = .25 / c
        x = (k - g) * c
        y = (d - h) * c
        z = (e - a) * c
    elif c > f and c > b:
        c = 2 * np.sqrt(1 + c - f - b)
        w = (k - g) / c
        x = .25 * c
        y = (a + e) / c
        z = (d + h) / c
    elif f > b:
        c = 2 * np.sqrt(1 + f - c - b)
        w = (d - h) / c
        x = (a + e) / c
        y = .25 * c
        z = (g + k) / c
    else:
        c = 2 * np.sqrt(1 + b - c - f)
        w = (e - a) / c
        x = (d + h) / c
        y = (g + k) / c
        z = .25 * c
    quaternion = np.array([w,x,y,z],dtype=np.float32)
    return quaternion

def matrix2axis(m,with_magnitude=False):
    quaternion = matrix2quaternion(m)
    theta = np.arccos(quaternion[0])
    theta_s = np.sin(theta)
    if theta_s>1e-3:
        u = [q/theta_s for q in quaternion[1:]]
    else:
        u = [1.0,0.0,0.0]
    theta *=2
    if with_magnitude:
        axis_angle = np.array(u,dtype=np.float32)*theta
    else:
        axis_angle = np.array(u+[theta],dtype=np.float32)
    return axis_angle

def translateOrientation(R):
    return matrix2axis(R)

def getMatrix(R):
    return np.array([[R[0], R[1], R[2]],[R[3], R[4], R[5]],[R[6], R[7], R[8]]])

def getMatrix4(R):
    return np.array([[R[0], R[1], R[2], R[4]], [R[4], R[5], R[6], R[7]], [R[8], R[9], R[10], R[11]], [R[12], R[13], R[14], R[15]]])

def getArray(m):
    return [float(m[0,0]), float(m[0,1]), float(m[0,2]), float(m[1,0]), float(m[1,1]), float(m[1,2]), float(m[2,0]), float(m[2,1]), float(m[2,2])]

def getAxisOfRotation(m):
    diff_10_01 = (float(m[1,0]) - float(m[0,1]))
    diff_02_20 = (float(m[0,2]) - float(m[2,0]))
    diff_21_12 = (float(m[2,1]) - float(m[1,2]))

    # print(diff_10_01, diff_02_20, diff_21_12)

    d = math.sqrt(diff_21_12 * diff_21_12 + diff_02_20 * diff_02_20 + diff_10_01 * diff_10_01)

    x = diff_21_12 / d
    y = diff_02_20 / d
    return [x, y]