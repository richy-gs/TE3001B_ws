import scipy.spatial.transform


def slerp(q1, q2, t):
    q1 = scipy.spatial.transform.Rotation.from_quat(q1)
    q2 = scipy.spatial.transform.Rotation.from_quat(q2)

    slerp_interpolator = scipy.spatial.transform.Slerp(
        [0, 1], scipy.spatial.transform.Rotation.from_quat([q1.as_quat(), q2.as_quat()])
    )

    return slerp_interpolator([t]).as_matrix()


# Ejemplo de uso
q1 = [0, 0, 0, 1]  # Identidad
q2 = [0.7071, 0, 0, 0.7071]  # Rotación de 90 grados en X
t = 0.5  # Punto intermedio

R_interpolada = slerp(q1, q2, t)
print("Matriz de rotación interpolada:")
print(R_interpolada)
