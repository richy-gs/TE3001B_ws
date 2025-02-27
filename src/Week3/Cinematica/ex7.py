import numpy as np


def euler_to_rotation(alpha, beta, gamma):
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

    return Rz @ Ry @ Rx


def rodrigues_rotation(axis, theta):
    axis = axis / np.linalg.norm(axis)  # Normalize axis
    omega_hat = np.array(
        [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
    )

    I = np.eye(3)
    R = I + np.sin(theta) * omega_hat + (1 - np.cos(theta)) * (omega_hat @ omega_hat)
    return R


# Example usage
R_euler = euler_to_rotation(np.pi / 2, np.pi / 4, 0)
print("Euler Rotation Matrix:")
print(R_euler)

axis = np.array([0, 0, 1])
theta = np.pi / 3
R_rodrigues = rodrigues_rotation(axis, theta)
print("Rodrigues Rotation Matrix:")
print(R_rodrigues)
