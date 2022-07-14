import numpy as np


def calc_N_unnormalized(v0, v1, v2):
    e0 = v1 - v0
    e1 = v2 - v1
    # print(f"e0 {e0} e1 {e1}")

    N = np.cross(e0, e1)
    # print(f"N {N} {type(N)}")
    # N_norm = np.linalg.norm(N)
    # print(f"N_norm {N_norm}")
    # N = N / N_norm
    # print(f"after N {N}")
    return N


def gen_pts():
    v0 = np.random.rand(3)
    v1 = np.random.rand(3)
    v2 = np.random.rand(3)
    v3 = np.random.rand(3)
    return v0, v1, v2, v3


def gen_edge(v0, v1, v2, v3):
    '''
            v2
    v0       |      v3
            v1
    '''
    # 1. n0, n1
    # 2. e
    e = v2 - v1

    n0 = calc_N_unnormalized(v0, v1, v2)
    n1 = calc_N_unnormalized(v1, v3, v2)
    return n0, n1, e


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


def normalize(x):
    return x / np.linalg.norm(x)


from arctanc_deriv import calc_arctan, calc_Darctan_Dcos_ana, calc_Darctan_Dsin_ana
'''
part1 = I'T
part2 = - 1.0 / sqrt(1 - y ** 2)
part3 = - bar_n1 \times e
part4 = -1.0 / sqrt(1 - x ** 2)
part5 = bar_n1
'''


def D_dihedral_angle_n0(n0, n1, e_):
    bar_n0 = normalize(n0)
    bar_n1 = normalize(n1)
    bar_e = normalize(e_)

    x = np.dot(bar_n0, bar_n1)
    y = np.dot(bar_e, np.cross(bar_n0, bar_n1))

    part1 = calc_Darctan_Dcos_ana(x, y)
    part2 = DnormedX_DX_ana_new(n0).T
    deriv = part1 * np.matmul(part2, bar_n1)
    return deriv


def D_dihedral_angle_n1(n0, n1, e_):
    bar_n0 = normalize(n0)
    bar_n1 = normalize(n1)
    bar_e = normalize(e_)

    x = np.dot(bar_n0, bar_n1)
    y = np.dot(bar_e, np.cross(bar_n0, bar_n1))

    part1 = calc_Darctan_Dcos_ana(x, y)
    part2 = DnormedX_DX_ana_new(n1).T
    deriv = part1 * np.matmul(part2, bar_n0)
    return deriv


def D_dihedral_angle_e(n0, n1, e_):
    bar_n0 = normalize(n0)
    bar_n1 = normalize(n1)
    bar_e = normalize(e_)

    x = np.dot(bar_n0, bar_n1)
    y = np.dot(bar_e, np.cross(bar_n0, bar_n1))
    # print(f"e {e_} bar_e {bar_e}")
    # print(f"x {x} y {y}")
    part1 = calc_Darctan_Dsin_ana(x, y)
    # print(f"part1 {part1}")
    part2 = DnormedX_DX_ana_new(e_).T
    # print(f"part2 {part2}")
    deriv = part1 * np.matmul(part2, np.cross(bar_n0, bar_n1))
    # print(f"deriv {deriv}")
    return deriv


def calc_dihedral(n0, n1, e):
    bar_n0 = normalize(n0)
    bar_n1 = normalize(n1)
    bar_e = normalize(e)

    cos = np.dot(bar_n0, bar_n1)
    sin = np.dot(bar_e, np.cross(bar_n0, bar_n1))
    theta = np.arctan2(sin, cos)
    return theta


def D_dihedral_angle_n0_num(n0, n1, e):
    theta_old = calc_dihedral(n0, n1, e)
    deriv_num = np.zeros(3)
    eps = 1e-5
    for i in range(3):
        n0[i] += eps
        theta_new = calc_dihedral(n0, n1, e)
        deriv_num[i] = (theta_new - theta_old) / eps

        n0[i] -= eps
    return deriv_num


def D_dihedral_angle_n1_num(n0, n1, e):
    theta_old = calc_dihedral(n0, n1, e)
    deriv_num = np.zeros(3)
    eps = 1e-5
    for i in range(3):
        n1[i] += eps
        theta_new = calc_dihedral(n0, n1, e)
        deriv_num[i] = (theta_new - theta_old) / eps

        n1[i] -= eps
    return deriv_num


def D_dihedral_angle_e_num(n0, n1, e):
    theta_old = calc_dihedral(n0, n1, e)
    deriv_num = np.zeros(3)
    eps = 1e-5
    for i in range(3):
        e[i] += eps
        theta_new = calc_dihedral(n0, n1, e)
        deriv_num[i] = (theta_new - theta_old) / eps

        e[i] -= eps
    return deriv_num


from skew import vecToSkewMat


def D_dihedral_dv0_ana(v0, v1, v2, v3):
    '''
    d theta / d n0
    '''
    n0, n1, e = gen_edge(v0, v1, v2, v3)
    dtheta_dn0 = D_dihedral_angle_n0(n0, n1, e)
    print(f"[calc] dtheta_dn0 = {dtheta_dn0}")
    '''
    d n0 / d v0
    '''
    res = np.matmul(vecToSkewMat(v2 - v1).T, dtheta_dn0)
    return res


def D_dihedral_dv0_num(v0, v1, v2, v3):
    dtheta_dv0 = np.zeros(3)
    eps = 1e-5
    theta_old = calc_dihedral(*gen_edge(v0, v1, v2, v3))
    for i in range(3):
        v0[i] += eps
        n0, n1, e = gen_edge(v0, v1, v2, v3)

        theta_new = calc_dihedral(n0, n1, e)
        dtheta_dv0[i] = (theta_new - theta_old) / eps
        v0[i] -= eps
    return dtheta_dv0

def calc_Dnormal_Dv0_ana(v0, v1, v2):
    return vecToSkewMat( v2 - v1)

def calc_Dnormal_Dv0_ana(v0, v1, v2):
    old_N = calc_N_unnormalized()

if __name__ == "__main__":
    np.random.seed(0)
    '''
            v2
    v0       |      v3
            v1
    '''
    for _ in range(1):
        v0, v1, v2, v3 = gen_pts()
        n0, n1, e = gen_edge(v0, v1, v2, v3)
        print(f"v0 {v0}")
        print(f"v1 {v1}")
        print(f"v2 {v2}")
        print(f"v3 {v3}")
        # print(f"n0 {n0}")
        # print(f"n1 {n1}")
        # print(f"e {e}")
        # n0 *= 2
        # n1 *= 2
        # e *= 2
        angle = calc_dihedral(n0, n1, e)
        print(f"angle {angle}")
        # 1. d \beta / d n0
        # if True:
        #     deriv_ana = D_dihedral_angle_n0(n0, n1, e)
        #     deriv_num = D_dihedral_angle_n0_num(n0, n1, e)
        #     print(f"deriv_ana n0 {deriv_ana}")
        #     print(f"deriv_num n0 {deriv_num}")
        #     assert np.linalg.norm(deriv_ana - deriv_num) < 1e-1
        
        # # # 2. d \beta / d n1
        # if True:
        #     deriv_ana = D_dihedral_angle_n1(n0, n1, e)
        #     deriv_num = D_dihedral_angle_n1_num(n0, n1, e)
        #     print(f"deriv_ana n1 {deriv_ana}")
        #     print(f"deriv_num n1 {deriv_num}")
        #     assert np.linalg.norm(deriv_ana - deriv_num) < 1e-1
        # exit()
        # # 2. d \beta / d e
        # if True:
        #     deriv_ana = D_dihedral_angle_e(n0, n1, e)
        #     deriv_num = D_dihedral_angle_e_num(n0, n1, e)
        #     print(f"deriv_ana e {deriv_ana}")
        #     print(f"deriv_num e {deriv_num}")
        #     assert np.linalg.norm(deriv_ana - deriv_num) < 1e-1

        # 1. get the  d \theta / d x0 (vector)
        # '''
        # d theta / dx0 = 
        # '''
        dtheta_dv0_num = D_dihedral_dv0_num(v0, v1, v2, v3)
        dtheta_dv0_ana = D_dihedral_dv0_ana(v0, v1, v2, v3)
        print(f"dtheta_dv0_ana {dtheta_dv0_ana}")
        print(f"n0 {n0 / np.linalg.norm(n0)}")
        print(f"ana {dtheta_dv0_ana}")
        print(f"num {dtheta_dv0_num}")
        # 2. get the u1 vector (normal of triangle 0)
        # exit()