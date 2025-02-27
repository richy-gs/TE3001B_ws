import numpy as np


def quaternion_to_matrix(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ]
    )


# Ejemplo de uso
q = (0.5, 0.5, 0.5, 0.5)  # Cuaternión dado en el problema
R = quaternion_to_matrix(q)

print("Matriz de rotación:")
print(R)
