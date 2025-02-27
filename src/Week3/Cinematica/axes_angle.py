import numpy as np


def matrix_mult(A, B):
    return np.dot(A, B)


def skew_symmetric(omega):
    return np.array(
        [[0, -omega[2], omega[1]], [omega[2], 0, -omega[0]], [-omega[1], omega[0], 0]]
    )


def axis_angle_rotation(p, omega, theta):
    omega_hat = skew_symmetric(omega)
    identity_matrix = np.identity(3)
    R = (
        identity_matrix
        + np.sin(theta) * omega_hat
        + (1 - np.cos(theta)) * matrix_mult(omega_hat, omega_hat)
    )
    return matrix_mult(R, p)


# Ejemplo de uso
p = np.array([1, 0, 0])  # Punto en coordenadas (x, y, z)
omega = np.array([0, 0, 1])  # Eje de rotación
theta = np.radians(45)  # Ángulo de rotación en radianes
p_rotado = axis_angle_rotation(p, omega, theta)
print("Punto rotado:", p_rotado)
