import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np


def forward_kinematics(theta1, theta2, theta3, link_lengths):
    """Calcula la posición del extremo del brazo"""
    l1, l2, l3 = link_lengths

    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)

    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)

    x3 = x2 + l3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + l3 * np.sin(theta1 + theta2 + theta3)

    return np.array([[0, x1, x2, x3], [0, y1, y2, y3]])


# Parámetros
link_lengths = [1, 1, 1]  # Longitudes de los eslabones
obstacle = np.array([0.5, 0.5])  # Punto a evitar
resolution = 20

theta_range = np.linspace(0, 2 * np.pi, resolution)
c_space = []
frames = []

fig, ax = plt.subplots()
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
(line,) = ax.plot([], [], "o-", lw=2)
ax.plot(obstacle[0], obstacle[1], "ro", label="Obstáculo")
ax.legend()


def init():
    line.set_data([], [])
    return (line,)


def update(frame):
    positions = forward_kinematics(frame[0], frame[1], frame[2], link_lengths)
    line.set_data(positions[0], positions[1])
    return (line,)


for theta1 in theta_range:
    for theta2 in theta_range:
        for theta3 in theta_range:
            positions = forward_kinematics(theta1, theta2, theta3, link_lengths)
            distances = np.linalg.norm(positions.T - obstacle, axis=1)
            if np.any(distances < 0.2):
                c_space.append([theta1, theta2, theta3, 1])
            else:
                c_space.append([theta1, theta2, theta3, 0])
                frames.append([theta1, theta2, theta3])

c_space = np.array(c_space)

fig3d = plt.figure()
ax3d = fig3d.add_subplot(111, projection="3d")
ax3d.scatter(
    c_space[c_space[:, 3] == 0, 0],
    c_space[c_space[:, 3] == 0, 1],
    c_space[c_space[:, 3] == 0, 2],
    c="b",
    alpha=0.0,
    label="Válido",
)
ax3d.scatter(
    c_space[c_space[:, 3] == 1, 0],
    c_space[c_space[:, 3] == 1, 1],
    c_space[c_space[:, 3] == 1, 2],
    c="r",
    label="Inválido",
)
ax3d.set_xlabel("Theta 1")
ax3d.set_ylabel("Theta 2")
ax3d.set_zlabel("Theta 3")
ax3d.legend()

ani = animation.FuncAnimation(
    fig, update, frames=frames, init_func=init, blit=True, interval=50
)
plt.show()
