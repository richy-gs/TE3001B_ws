from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_node = Node(
        name="motor_sys",
        package="motor_control",
        executable="dc_motor",
        emulate_tty=True,
        output="screen",
        parameters=[
            {
                "sample_time": 0.018,  # Paso de muestreo [s]
                # Parámetros eléctricos del motor
                "armature_inductance_La": 3.3e-3,  # Inductancia [H]
                "armature_resistance_Ra": 0.5,  # Resistencia de armadura [Ohm]
                "motor_const_Ka": 0.018,  # Constante de torque [N·m/A]
                "back_emf_const_Kb": 0.06,  # Constante de FEM [V·s/rad]
                # Parámetros mecánicos
                "motor_inertia_Jm": 0.0005,  # Inercia del rotor [kg·m^2]
                "motor_friction_b": 0.0027,  # Fricción viscosa [N·m·s/rad]
                "load_torque_tau_c": 0.0,  # Torque de carga [N·m]
            }
        ],
    )

    # sp_node = Node(name="sp_gen",
    #                   package='motor_control',
    #                   executable='set_point',
    #                   emulate_tty=True,
    #                   output='screen',
    #                   parameters = [{
    #                       'signal_type': "square",
    #                       'amplitude': 2.0,
    #                       'omega': 1.0,
    #                   }]
    #                   )

    ctrl_node = Node(
        name="ctrl",
        package="motor_control",
        executable="ctrl",
        emulate_tty=True,
        output="screen",
        parameters=[
            {
                "sample_time": 0.018,  # Mismo paso de muestreo que el motor
                "kP": 80.0,
                "kI": 0.5,
                "kD": 20.0,
                # --- Parámetros del motor para Feedback Linearization o su uso interno ---
                "La": 3.3e-3,
                "Ra": 0.5,
                "Ka": 0.018,
                "Kb": 0.06,
                "Jm": 0.0005,
                "b": 0.0027,
                "tau_c": 0.0,
                # Límites de saturación (si tu controlador linealizado los necesita)
                "vmin": -6.0,
                "vmax": 6.0,
            }
        ],
    )

    l_d = LaunchDescription([motor_node, ctrl_node])

    return l_d
