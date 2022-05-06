import numpy as np


def calc_arctan(cos, sin):
    val = np.arctan2(sin, cos)
    return val


def calc_Darctan_Dcos_ana(cos, sin):
    sin_t = 1 if np.sin(calc_arctan(cos, sin)) >= 0 else -1
    return -sin_t / np.sqrt(1 - cos * cos)


def calc_Darctan_Dsin_ana(cos, sin):
    cos_t = 1 if np.cos(calc_arctan(cos, sin)) >= 0 else -1
    return cos_t / np.sqrt(1 - sin * sin)


def calc_Darctan_Dcos_num(cos, sin):
    old_theta = calc_arctan(cos, sin)
    new_cos = cos + eps
    new_sin = np.sqrt(1 - new_cos * new_cos) * np.sign(sin)
    new_theta = calc_arctan(new_cos, new_sin)
    return (new_theta - old_theta) / eps


def calc_Darctan_Dsin_num(cos, sin):
    old_theta = calc_arctan(cos, sin)
    new_sin = sin + eps
    new_cos = np.sqrt(1 - new_sin * new_sin) * np.sign(cos)
    new_theta = calc_arctan(new_cos, new_sin)
    return (new_theta - old_theta) / eps


def rel_err(a, b):
    if np.abs(a) > eps:
        return np.abs((a - b) / a)
    else:
        return np.abs((a - b) / (a + eps))


if __name__ == "__main__":
    eps = 1e-9
    for _ in range(1000):
        cos = np.random.rand() * 2 - 1
        sin = np.sqrt(1 - cos * cos) * np.sign(np.random.rand() - 0.5)
        theta = calc_arctan(cos, sin)

        deriv_cos_ana = calc_Darctan_Dcos_ana(cos, sin)
        deriv_cos_num = calc_Darctan_Dcos_num(cos, sin)
        print("cos", deriv_cos_ana, deriv_cos_num)
        assert rel_err(
            deriv_cos_ana,
            deriv_cos_num) < 1e-2, f"{deriv_cos_ana} {deriv_cos_num}"

        deriv_sin_ana = calc_Darctan_Dsin_ana(cos, sin)
        deriv_sin_num = calc_Darctan_Dsin_num(cos, sin)
        print("sin", deriv_sin_ana, deriv_sin_num)
        assert rel_err(
            deriv_sin_ana,
            deriv_sin_num) < 1e-2, f"{deriv_sin_ana} {deriv_sin_num}"
