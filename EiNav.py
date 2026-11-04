"""Point d'entrée du robot. Webots impose que ce fichier porte le nom du dossier du contrôleur."""
from __future__ import annotations

import subprocess
import time
from typing import Optional, Tuple

from connections.base import RobotConnection
from connections.serialpi import Serial
from planning.direction import Direction
from planning.navigation import Position
from robot.state_machine import run_state_machine
from robot.states import Mode
from robot.telemetry import TelemetryServer, start_dashboard, start_telemetry

# Simulation flag
SIMULATION = False

START_POS: Position = (0, 1)
END_POS: Position = (4, 4)
START_DIR = Direction.SOUTH


def create_environment(simulation: bool) -> Tuple[RobotConnection, Serial]:
    if simulation:
        from connections.webots_connection import WebotsConnection

        webots = WebotsConnection()
        return webots, Serial(robot=webots.robot)

    from connections.real_robot_connection import RealRobotConnection

    return RealRobotConnection(), Serial(robot=None)


def main() -> None:
    telemetry: Optional[TelemetryServer] = start_telemetry()
    dashboard: Optional["subprocess.Popen[bytes]"] = None if SIMULATION else start_dashboard()
    try:
        connection, serial = create_environment(SIMULATION)
        try:
            try:
                run_state_machine(connection, serial, START_POS, END_POS, START_DIR, Mode.GRID,
                                  telemetry=telemetry)
            finally:
                serial.disconnect()

            if not SIMULATION:
                # Les moteurs sont coupés : le tableau de bord reste consultable.
                print("Mission terminée. Ctrl+C pour fermer le tableau de bord.")
                while True:
                    time.sleep(1.0)
        finally:
            connection.close()
    except KeyboardInterrupt:
        pass
    finally:
        if telemetry is not None:
            telemetry.close()
        if dashboard is not None:
            dashboard.terminate()


if __name__ == "__main__":
    main()
