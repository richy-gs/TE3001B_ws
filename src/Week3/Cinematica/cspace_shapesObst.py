import matplotlib.pyplot as plt
import numpy as np


def forward_kinematics(theta1, theta2, theta3, l1=1, l2=1, l3=1):
    """
    Calcula la posición del efector final dados los ángulos de las tres articulaciones.
    """
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)

    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)

    x3 = x2 + l3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + l3 * np.sin(theta1 + theta2 + theta3)
    z3 = 0  # Suponemos un movimiento en el plano XY

    return x3, y3, z3


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
        obstacles.append((x[i], y[i], z[i], 0.05))  # Cada punto con un pequeño radio

    return obstacles


def check_collision(x, y, z, obstacles):
    """
    Verifica si la posición (x, y, z) del efector final está dentro de una zona de colisión.
    """
    for obs in obstacles:
        ox, oy, oz, radius = obs
        if (x - ox) ** 2 + (y - oy) ** 2 + (z - oz) ** 2 <= radius**2:
            return True
    return False


def generate_config_space(resolution=50, shape="cube", obstacle_position=(0, 0, 0)):
    """
    Genera el espacio de configuración del robot discretizando los ángulos y considerando la forma de los obstáculos.
    """
    theta_range = np.linspace(-np.pi, np.pi, resolution)
    obstacles = generate_obstacles(shape, position=obstacle_position)
    config_space = []

    for t1 in theta_range:
        for t2 in theta_range:
            for t3 in theta_range:
                x, y, z = forward_kinematics(t1, t2, t3)
                collision = check_collision(x, y, z, obstacles)
                config_space.append((t1, t2, t3, collision))

    return np.array(config_space), obstacles


def plot_config_space(config_space, obstacles):
    """
    Grafica el espacio de configuración del robot en 3D, mostrando colisiones en rojo.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    non_collision = config_space[config_space[:, 3] == False]
    collision = config_space[config_space[:, 3] == True]

    ax.scatter(
        non_collision[:, 0],
        non_collision[:, 1],
        non_collision[:, 2],
        c="blue",
        s=5,
        alpha=0.01,
        label="Sin colisión",
    )
    ax.scatter(
        collision[:, 0],
        collision[:, 1],
        collision[:, 2],
        c="red",
        s=5,
        label="Colisión",
    )

    # Dibujar los obstáculos
    obstacle_x = [obs[0] for obs in obstacles]
    obstacle_y = [obs[1] for obs in obstacles]
    obstacle_z = [obs[2] for obs in obstacles]
    ax.scatter(obstacle_x, obstacle_y, obstacle_z, c="black", s=2, label="Obstáculos")

    ax.set_xlabel("Theta1")
    ax.set_ylabel("Theta2")
    ax.set_zlabel("Theta3")
    plt.title("Espacio de Configuración con Obstáculos en 3D")
    plt.legend()
    plt.show()


# Seleccionar la forma del obstáculo y su posición
shape = "cube"  # Cambiar a "sphere" o "cylinder" según necesidad
obstacle_position = (0.3, 0.3, 0.3)  # Posición del obstáculo en el espacio
config_space, obstacles = generate_config_space(
    resolution=50, shape=shape, obstacle_position=obstacle_position
)
plot_config_space(config_space, obstacles)
