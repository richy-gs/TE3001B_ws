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


def generate_obstacles(shape="cube", num_points=500, position=(0, 0, 0)):
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
        r = 0.1  # Radio del cilindro
        x = r * np.cos(theta) + ox
        y = r * np.sin(theta) + oy
        z = h + oz
    else:
        raise ValueError("Forma no reconocida. Usa 'cube', 'sphere' o 'cylinder'.")

    for i in range(num_points):
        obstacles.append((x[i], y[i], z[i]))

    return obstacles


def check_collision(points, obstacles):
    """
    Verifica si alguna parte del brazo colisiona con un obstáculo.
    """
    for px, py, pz in points:
        for ox, oy, oz in obstacles:
            if (px - ox) ** 2 + (py - oy) ** 2 + (
                pz - oz
            ) ** 2 < 0.02:  # Umbral de colisión
                return True
    return False


def set_axes_equal(ax):
    """
    Ajusta los ejes para que tengan la misma escala en X, Y y Z.
    """
    limits = np.array([ax.get_xlim(), ax.get_ylim(), ax.get_zlim()])
    min_limit = np.min(limits)
    max_limit = np.max(limits)
    ax.set_xlim(min_limit, max_limit)
    ax.set_ylim(min_limit, max_limit)
    ax.set_zlim(min_limit, max_limit)


def plot_robot_and_obstacles(theta1, theta2, theta3, obstacles):
    """
    Grafica el brazo robótico en 3D junto con los obstáculos y el espacio de configuración.
    """
    points = forward_kinematics(theta1, theta2, theta3)

    fig = plt.figure(figsize=(12, 6))
    ax1 = fig.add_subplot(121, projection="3d")
    ax2 = fig.add_subplot(122, projection="3d")

    # Dibujar el brazo robótico
    xs, ys, zs = zip(*points)
    ax1.plot(xs, ys, zs, "-o", markersize=8, label="Brazo robótico", color="blue")

    # Dibujar el efector final
    ax1.scatter(xs[-1], ys[-1], zs[-1], color="magenta", s=100, label="Efector final")

    # Dibujar los obstáculos
    obstacle_x = [obs[0] for obs in obstacles]
    obstacle_y = [obs[1] for obs in obstacles]
    obstacle_z = [obs[2] for obs in obstacles]
    ax1.scatter(obstacle_x, obstacle_y, obstacle_z, c="red", s=5, label="Obstáculo")

    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.set_zlabel("Z")
    ax1.set_title("Simulación del Brazo Robótico con Obstáculos")
    ax1.legend()
    set_axes_equal(ax1)

    # Espacio de Configuración con verificación de colisión
    theta_range = np.linspace(-np.pi, np.pi, 25)
    config_space = []

    for t1 in theta_range:
        for t2 in theta_range:
            for t3 in theta_range:
                points = forward_kinematics(t1, t2, t3)
                collision = check_collision(points, obstacles)
                config_space.append((t1, t2, t3, collision))

    config_space = np.array(config_space)

    non_collision = config_space[config_space[:, 3] == False]
    collision = config_space[config_space[:, 3] == True]

    ax2.scatter(
        non_collision[:, 0],
        non_collision[:, 1],
        non_collision[:, 2],
        c="blue",
        s=2,
        alpha=0.01,
        label="Sin colisión",
    )
    ax2.scatter(
        collision[:, 0],
        collision[:, 1],
        collision[:, 2],
        c="red",
        s=2,
        alpha=0.5,
        label="Colisión",
    )

    ax2.set_xlabel("Theta1")
    ax2.set_ylabel("Theta2")
    ax2.set_zlabel("Theta3")
    ax2.set_title("Espacio de Configuración")
    ax2.legend()

    plt.show()


# Configuración inicial
shape = "cylinder"  # Cambiar a "cube", "sphere" o "cylinder"
obstacle_position = (0.5, 0.5, 0)  # Posición del obstáculo
obstacles = generate_obstacles(shape, position=obstacle_position)

theta1, theta2, theta3 = 0, 0, 0  # Ángulos de prueba
plot_robot_and_obstacles(theta1, theta2, theta3, obstacles)
