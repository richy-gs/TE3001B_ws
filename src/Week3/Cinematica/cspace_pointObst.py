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

    return x3, y3


def check_collision(x, y, obstacles):
    """
    Verifica si la posición (x, y) del efector final está dentro de una zona de colisión.
    """
    for obs in obstacles:
        ox, oy, radius = obs
        if (x - ox) ** 2 + (y - oy) ** 2 <= radius**2:
            return True
    return False


def generate_config_space(
    resolution=30, obstacles=[(0.5, 0.5, 0.2), (-0.5, -0.5, 0.2)]
):
    """
    Genera el espacio de configuración del robot discretizando los ángulos.
    """
    theta_range = np.linspace(-np.pi, np.pi, resolution)
    config_space = []

    for t1 in theta_range:
        for t2 in theta_range:
            for t3 in theta_range:
                x, y = forward_kinematics(t1, t2, t3)
                collision = check_collision(x, y, obstacles)
                config_space.append((t1, t2, t3, collision))

    return np.array(config_space)


def plot_config_space(config_space):
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
        alpha=0.0,
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

    ax.set_xlabel("Theta1")
    ax.set_ylabel("Theta2")
    ax.set_zlabel("Theta3")
    plt.title("Espacio de Configuración con Detección de Colisiones")
    plt.legend()
    plt.show()


# Generar el espacio de configuración con obstáculos
config_space = generate_config_space(resolution=50)
plot_config_space(config_space)
