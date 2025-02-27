import numpy as np


def matrix_mult(A, B):
    return np.dot(A, B)


def rotation_matrix(axis, angle):
    c, s = np.cos(angle), np.sin(angle)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    elif axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis == "z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def combine_rotations(rotations):
    R = np.identity(3)
    for rotation in rotations:
        R = matrix_mult(R, rotation)
    return R


# Definir las rotaciones
rot_z = rotation_matrix("z", np.pi / 4)  # Rotación de π/4 alrededor de Z
rot_x = rotation_matrix("x", np.pi / 6)  # Rotación de π/6 alrededor de X
rot_y = rotation_matrix("y", np.pi / 3)  # Rotación de π/3 alrededor de Y

# Combinar las rotaciones en orden z -> x -> y
R_combined = combine_rotations([rot_z, rot_x, rot_y])

# Punto a transformar
p = np.array([1, 0, 0])
p_transformed = matrix_mult(R_combined, p)

print("Matriz de rotación combinada:")
print(R_combined)
print("Punto transformado:", p_transformed)
