import numpy as np


def matrix_mult(A, B):
    return np.dot(A, B)


def skew_symmetric(axis):
    return np.array(
        [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
    )


def matrix_exponential(omega_hat, angle):
    identity_matrix = np.identity(3)
    return (
        identity_matrix
        + np.sin(angle) * omega_hat
        + (1 - np.cos(angle)) * matrix_mult(omega_hat, omega_hat)
    )


def axis_angle_to_matrix(axis, angle):
    omega_hat = skew_symmetric(axis)
    return matrix_exponential(omega_hat, angle)


# Definir el eje y el ángulo de rotación
eje = np.array([1, 0, 0])  # Rotación alrededor del eje X
angulo = np.pi / 2  # Ángulo de rotación en radianes

# Calcular la matriz de rotación
R = axis_angle_to_matrix(eje, angulo)

# Punto a rotar
p = np.array([1, 0, 0])
p_rotado = matrix_mult(R, p)

print("Matriz de rotación R:")
print(R)
print("Punto rotado:", p_rotado)
