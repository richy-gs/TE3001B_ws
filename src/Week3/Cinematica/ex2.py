import numpy as np


def matrix_mult(A, B):
    return np.dot(A, B)


def matrix_exponential(omega_hat, theta):
    identity_matrix = np.identity(3)
    return (
        identity_matrix
        + np.sin(theta) * omega_hat
        + (1 - np.cos(theta)) * matrix_mult(omega_hat, omega_hat)
    )


# Definición del generador omega_hat para rotación en Z
omega_hat = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])

theta = np.pi / 4  # Ángulo de rotación en radianes
R_exp = matrix_exponential(omega_hat, theta)

# Punto a rotar
p = np.array([1, 0, 0])
p_rotado = matrix_mult(R_exp, p)

print("Matriz de rotación R:")
print(R_exp)
print("Punto rotado:", p_rotado)
