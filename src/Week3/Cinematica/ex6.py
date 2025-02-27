def quaternion_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return (w, x, y, z)


# Ejemplo de uso
q1 = (0.7071, 0.7071, 0, 0)  # Cuaternión de rotación
q2 = (0.7071, 0, 0.7071, 0)  # Otro cuaternión de rotación

q_composed = quaternion_mult(q1, q2)
print("Cuaternión compuesto:", q_composed)
