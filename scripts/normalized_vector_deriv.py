import numpy as np
def DnormedX_DX_ana_old(x):
    deriv = np.zeros((3, 3))
    x_norm = np.linalg.norm(x)
    x_normed = x / x_norm
    for i in range(3):
        for j in range(3):
            deriv[i,
                  j] = ((i == j) * x_norm - x[i] * x_normed[j]) / (x_norm**2)
    return deriv


def DnormedX_DX_ana_new(x):
    deriv = np.identity(3)
    x_norm = np.linalg.norm(x)
    x_normed = np.atleast_2d(x / x_norm)
    deriv -= np.matmul(x_normed.T, x_normed)
    deriv /= x_norm
    return deriv


def DnormedX_DX_num(x):
    deriv = np.zeros((3, 3))
    eps = 1e-6
    x_normed = x / np.linalg.norm(x)
    for i in range(3):
        x[i] += eps
        x_normed_new = x / np.linalg.norm(x)
        deriv[:, i] = (x_normed_new - x_normed) / eps
        x[i] -= eps
    return deriv


if __name__ == "__main__":
    np.random.seed(0)
    x = np.array([2, 3.4, 4])
    ana = DnormedX_DX_ana_new(x)
    num = DnormedX_DX_num(x)
    print("x", x)
    print("ana", ana)
    print("num", num)