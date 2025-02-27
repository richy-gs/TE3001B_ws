import numpy as np


def matrix_mult(A, B):
    return np.dot(A, B)


def euler_rotation(p, alpha, beta, gamma):
    Rz = np.array(
        [
            [np.cos(alpha), -np.sin(alpha), 0],
            [np.sin(alpha), np.cos(alpha), 0],
            [0, 0, 1],
        ]
    )

    Ry = np.array(
        [[np.cos(beta), 0, np.sin(beta)], [0, 1, 0], [-np.sin(beta), 0, np.cos(beta)]]
    )

    Rx = np.array(
        [
            [1, 0, 0],
            [0, np.cos(gamma), -np.sin(gamma)],
            [0, np.sin(gamma), np.cos(gamma)],
        ]
    )

    R = matrix_mult(Rz, matrix_mult(Ry, Rx))
    return matrix_mult(R, p)


# Ejemplo de uso
p = np.array([1, 0, 0])  # Punto en coordenadas (x, y, z)
alpha, beta, gamma = (
    np.radians(30),
    np.radians(45),
    np.radians(60),
)  # Ángulos en radianes
p_rotado = euler_rotation(p, alpha, beta, gamma)
print("Punto rotado:", p_rotado)
