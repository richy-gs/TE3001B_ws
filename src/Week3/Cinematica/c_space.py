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


def generate_config_space(resolution=20):
    """
    Genera el espacio de configuración del robot discretizando los ángulos.
    """
    theta_range = np.linspace(-np.pi, np.pi, resolution)
    config_space = []

    for t1 in theta_range:
        for t2 in theta_range:
            for t3 in theta_range:
                x, y = forward_kinematics(t1, t2, t3)
                config_space.append((t1, t2, t3, x, y))

    return np.array(config_space)


def plot_config_space(config_space):
    """
    Grafica el espacio de configuración del robot en 3D.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        config_space[:, 0],
        config_space[:, 1],
        config_space[:, 2],
        c=config_space[:, 3],
        cmap="viridis",
        s=5,
    )
    ax.set_xlabel("Theta1")
    ax.set_ylabel("Theta2")
    ax.set_zlabel("Theta3")
    plt.title("Espacio de Configuración")
    plt.show()


# Generar el espacio de configuración
config_space = generate_config_space(resolution=45)
plot_config_space(config_space)
