import numpy as np


def vecToSkewMat(vec):
    assert len(vec) == 3
    w = np.zeros([3, 3])
    x = vec[0]
    y = vec[1]
    z = vec[2]
    w[0, 1] = -z
    w[0, 2] = y
    w[1, 0] = z
    w[1, 2] = -x
    w[2, 0] = -y
    w[2, 1] = x
    return w


def judgeMatSkew(mat):
    res = mat + mat.T
    return np.linalg.norm(res) < 1e-5


if __name__ == "__main__":
    veca = np.random.rand(3)
    vecb = np.random.rand(3)

    res0 = np.cross(veca, vecb)
    res1 = np.matmul(vecToSkewMat(veca), vecb)
    print(res0)
    print(res1)