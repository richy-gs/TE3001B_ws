import matplotlib.pyplot as plt
import numpy as np


def forward_kinematics(theta1, theta2, theta3, l1=1, l2=1, l3=1):
    """
    Calcula las posiciones de cada eslabón del brazo robótico.
    """
    x0, y0, z0 = 0, 0, 0

    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    z1 = 0

    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    z2 = 0

    x3 = x2 + l3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + l3 * np.sin(theta1 + theta2 + theta3)
    z3 = 0

    return [(x0, y0, z0), (x1, y1, z1), (x2, y2, z2), (x3, y3, z3)]


def generate_obstacles(shape="cube", num_points=1000, position=(0, 0, 0)):
    """
    Genera una nube de puntos que representa un obstáculo en forma de cubo, esfera o cilindro.
    """
    ox, oy, oz = position
    obstacles = []

    if shape == "cube":
        x = np.random.uniform(-0.5, 0.5, num_points) + ox
        y = np.random.uniform(-0.5, 0.5, num_points) + oy
        z = np.random.uniform(-0.5, 0.5, num_points) + oz
    elif shape == "sphere":
        theta = np.random.uniform(0, 2 * np.pi, num_points)
        phi = np.random.uniform(0, np.pi, num_points)
        r = 0.5  # Radio de la esfera
        x = r * np.sin(phi) * np.cos(theta) + ox
        y = r * np.sin(phi) * np.sin(theta) + oy
        z = r * np.cos(phi) + oz
    elif shape == "cylinder":
        theta = np.random.uniform(0, 2 * np.pi, num_points)
        h = np.random.uniform(-0.5, 0.5, num_points)
        r = 0.3  # Radio del cilindro
        x = r * np.cos(theta) + ox
        y = r * np.sin(theta) + oy
        z = h + oz
    else:
        raise ValueError("Forma no reconocida. Usa 'cube', 'sphere' o 'cylinder'.")

    for i in range(num_points):
        obstacles.append((x[i], y[i], z[i]))

    return obstacles


def plot_robot_and_obstacles(theta1, theta2, theta3, obstacles):
    """
    Grafica el brazo robótico en 3D junto con los obstáculos.
    """
    points = forward_kinematics(theta1, theta2, theta3)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Dibujar el brazo robótico
    xs, ys, zs = zip(*points)
    ax.plot(xs, ys, zs, "-o", markersize=8, label="Brazo robótico", color="blue")

    # Dibujar el efector final
    ax.scatter(xs[-1], ys[-1], zs[-1], color="magenta", s=100, label="Efector final")

    # Dibujar los obstáculos
    obstacle_x = [obs[0] for obs in obstacles]
    obstacle_y = [obs[1] for obs in obstacles]
    obstacle_z = [obs[2] for obs in obstacles]
    ax.scatter(obstacle_x, obstacle_y, obstacle_z, c="black", s=5, label="Obstáculo")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.title("Simulación del Brazo Robótico con Obstáculos")
    plt.legend()
    plt.show()


# Configuración inicial
shape = "sphere"  # Cambiar a "sphere" o "cylinder"
obstacle_position = (0.3, 0.3, 0.3)  # Posición del obstáculo
obstacles = generate_obstacles(shape, position=obstacle_position)

theta1, theta2, theta3 = np.pi / 4, -np.pi / 4, np.pi / 6  # Ángulos de prueba
plot_robot_and_obstacles(theta1, theta2, theta3, obstacles)
