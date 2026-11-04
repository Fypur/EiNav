"""Point d'entrée du robot : ouvre l'interface, puis attend ses ordres, moteurs coupés.

Webots impose que ce fichier porte le nom du dossier du contrôleur.
"""
from __future__ import annotations

from typing import Tuple

from connections.base import RobotConnection
from connections.serialpi import Serial
from robot.state_machine import serve_missions
from robot.telemetry import open_interface, start_dashboard, start_telemetry

# Simulation flag
SIMULATION = False


def create_environment(simulation: bool) -> Tuple[RobotConnection, Serial]:
    if simulation:
        from connections.webots_connection import WebotsConnection

        webots = WebotsConnection()
        return webots, Serial(robot=webots.robot)

    from connections.real_robot_connection import RealRobotConnection

    return RealRobotConnection(), Serial(robot=None)


def main() -> None:
    telemetry = start_telemetry()
    if telemetry is None:
        # Le robot ne reçoit ses ordres que par cette interface : inutile de continuer sans elle.
        raise SystemExit("Interface indisponible : le programme tourne-t-il déjà ?")

    dashboard = start_dashboard()
    open_interface(dashboard)
    try:
        connection, serial = create_environment(SIMULATION)
        try:
            try:
                serve_missions(connection, serial, telemetry)
            finally:
                serial.disconnect()
        finally:
            connection.close()
    except KeyboardInterrupt:
        pass
    finally:
        telemetry.close()
        if dashboard is not None:
            dashboard.terminate()


if __name__ == "__main__":
    main()
