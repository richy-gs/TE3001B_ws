import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize


def forward_kinematics(thetas, link_lengths):
    """Calcula la posición del extremo del brazo en 3D"""
    theta1, theta2, theta3 = thetas
    l1, l2, l3 = link_lengths

    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    z1 = 0

    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    z2 = z1 + l2 * np.sin(theta2)

    x3 = x2 + l3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + l3 * np.sin(theta1 + theta2 + theta3)
    z3 = z2 + l3 * np.sin(theta3)

    return np.array([x3, y3, z3]), np.array(
        [[0, x1, x2, x3], [0, y1, y2, y3], [0, z1, z2, z3]]
    )


def inverse_kinematics(target, link_lengths, initial_guess):
    """Encuentra los ángulos de articulación para alcanzar un objetivo usando optimización"""

    def objective(thetas):
        pos, _ = forward_kinematics(thetas, link_lengths)
        return np.linalg.norm(pos - target)

    result = minimize(
        objective, initial_guess, bounds=[(0, np.pi), (0, np.pi), (0, np.pi)]
    )
    return result.x if result.success else None


def interpolate_path(start, end, steps):
    """Genera una trayectoria lineal en el espacio de trabajo."""
    return np.linspace(start, end, steps)


# Parámetros
link_lengths = [1, 1, 1]
obstacle = np.array([0.5, 0.5, 0.5])
start_point = np.array([1, 1, 1])
end_point = np.array([-1, -1, 1])
steps = 20

# Generar trayectoria
trajectory = interpolate_path(start_point, end_point, steps)
angles_trajectory = []
initial_guess = [0.5, 0.5, 0.5]
for point in trajectory:
    angles = inverse_kinematics(point, link_lengths, initial_guess)
    if angles is not None:
        angles_trajectory.append(angles)
        initial_guess = angles

fig3d = plt.figure()
ax3d = fig3d.add_subplot(111, projection="3d")
(line,) = ax3d.plot([], [], [], "o-", lw=2)
ax3d.scatter(
    obstacle[0], obstacle[1], obstacle[2], c="r", marker="o", label="Obstáculo"
)
ax3d.scatter(
    start_point[0], start_point[1], start_point[2], c="g", marker="o", label="Inicio"
)
ax3d.scatter(end_point[0], end_point[1], end_point[2], c="b", marker="o", label="Fin")
ax3d.set_xlim(-3, 3)
ax3d.set_ylim(-3, 3)
ax3d.set_zlim(-3, 3)
ax3d.legend()


def init():
    line.set_data([], [])
    line.set_3d_properties([])
    return (line,)


def update(frame):
    _, positions = forward_kinematics(angles_trajectory[frame], link_lengths)
    line.set_data(positions[0], positions[1])
    line.set_3d_properties(positions[2])
    return (line,)


ani = animation.FuncAnimation(
    fig3d,
    update,
    frames=len(angles_trajectory),
    init_func=init,
    blit=True,
    interval=100,
)
plt.show()
