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
                "sample_time": 0.01,
                "sys_gain_K": 1.75,
                "sys_tau_T": 0.5,
                "initial_conditions": 0.0,
            }
        ],
    )

    sp_node = Node(
        name="sp_gen",
        package="motor_control",
        executable="set_point",
        emulate_tty=True,
        output="screen",
    )

    ctrl_node = Node(
        name="ctrl",
        package="motor_control",
        executable="controller",
        emulate_tty=True,
        output="screen",
        parameters=[
            {
                # Coeficiente Proporcional (Kp):
                # Es la ganancia que determina la respuesta proporcional al error actual.
                # A mayor Kp, el sistema responde más rápidamente al error, pero puede provocar oscilaciones.
                "Kp": 7.5,
                # Coeficiente Integral (Ki):
                # Es la ganancia que acumula el error a lo largo del tiempo para corregir errores pequeños o persistentes.
                # A mayor Ki, el sistema elimina más rápidamente los errores acumulados, pero puede inducir inestabilidad o sobreoscilaciones.
                "Ki": 7.0,
                # Coeficiente Derivativo (Kd):
                # Es la ganancia que responde a la tasa de cambio del error, actuando como un "freno" para reducir la velocidad de las oscilaciones.
                # A mayor Kd, el sistema reacciona más rápido ante cambios rápidos en el error, pero si es demasiado alto, puede hacer que el sistema sea demasiado sensible al ruido.
                "Kd": 0.01,
            }
        ],
    )

    l_d = LaunchDescription([motor_node, sp_node, ctrl_node])

    return l_d
