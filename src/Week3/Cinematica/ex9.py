import numpy as np


def quaternion_to_rotation(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)],
        ]
    )


# Example usage
q = [0.707, 0, 0.707, 0]  # Quaternion example
R = quaternion_to_rotation(q)
print("Rotation Matrix:")
print(R)
