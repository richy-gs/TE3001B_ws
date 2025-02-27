import numpy as np


def homogeneous_transform(R, t):
    T = np.eye(4)  # Crear una matriz identidad 4x4
    T[:3, :3] = R  # Insertar la matriz de rotación en la esquina superior izquierda
    T[:3, 3] = t  # Insertar el vector de traslación en la última columna
    return T


# Ejemplo de uso
R = np.eye(3)  # Matriz de rotación identidad

t = np.array([1, 2, 3])  # Vector de traslación

T = homogeneous_transform(R, t)
print("Matriz de transformación homogénea:")
print(T)
