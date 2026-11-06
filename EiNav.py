"""Programme principal du robot. On ouvre l'interface puis on attend les ordres.

Le fichier doit avoir le même nom que le dossier du contrôleur, sinon Webots ne le trouve pas.
"""
from __future__ import annotations

from typing import Tuple

from connections.base import RobotConnection
from connections.serialpi import Serial
from robot import tuning
from robot.state_machine import serve_missions
from robot.telemetry import open_interface, start_telemetry

# True pour lancer sous Webots
SIMULATION = False


def create_environment(simulation: bool) -> Tuple[RobotConnection, Serial]:
    if simulation:
        from connections.webots_connection import WebotsConnection

        webots = WebotsConnection()
        return webots, Serial(robot=webots.robot)

    from connections.real_robot_connection import RealRobotConnection

    return RealRobotConnection(), Serial(robot=None)


def main() -> None:
    tuning.load()
    telemetry = start_telemetry()
    if telemetry is None:
        # Sans interface on ne peut donner aucun ordre au robot, donc on arrête là
        raise SystemExit("Interface indisponible : le programme tourne-t-il déjà ?")

    open_interface()
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


if __name__ == "__main__":
    main()
