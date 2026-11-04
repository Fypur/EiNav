"""Machine à états du robot : suivi de ligne et navigation d'intersection en intersection."""
from __future__ import annotations

from typing import Optional

from connections.base import RobotConnection
from connections.serialpi import Serial
from planning.direction import Direction
from planning.navigation import Grid, Mission, Navigation, Position
from robot.control import Control
from robot.states import Mode, RobotState
from robot.telemetry import TelemetryServer, make_status
from vision.image_processor import ImageProcessor

APPROACH_DURATION = 0.9  # durée d'avance vers le centre, à vitesse d'avance constante


def get_turn_state(current_dir: Direction, next_dir: Direction) -> RobotState:
    directions = [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]
    difference = (directions.index(next_dir) - directions.index(current_dir)) % 4
    return {
        0: RobotState.FOLLOWING,
        1: RobotState.TURNING_MINUS_90,  # rotation droite
        2: RobotState.TURNING_180,  # demi-tour
        3: RobotState.TURNING_90,  # rotation gauche
    }[difference]


def run_state_machine(
    connection: RobotConnection,
    serial: Serial,
    start_pos: Position,
    end_pos: Position,
    robot_dir: Direction,
    mode: Mode = Mode.GRID,
    processor: Optional[ImageProcessor] = None,
    telemetry: Optional[TelemetryServer] = None,
) -> Mission:
    """Va de `start_pos` à `end_pos`, fait demi-tour et revient. Renvoie la mission (et ses flags)."""
    controller = Control(serial)
    processor = processor or ImageProcessor()

    grid = Grid(4)
    nav = Navigation()
    path = nav.generate_path(grid, start_pos, end_pos, robot_dir)
    if not path:
        controller.halt()
        raise ValueError(f"Impossible de calculer le chemin de {start_pos} à {end_pos}.")

    print("Chemin :", path)
    nav.display_path(grid, path)
    mission = Mission(path)

    robot_state = RobotState.FOLLOWING
    action_start_time = connection.get_time()
    approach_duration = APPROACH_DURATION
    # Empêche de détecter plusieurs fois la même intersection
    intersection_armed = True

    def publish(line_detected: bool = False, is_intersection: bool = False) -> None:
        if telemetry is not None:
            telemetry.publish(make_status(mode, robot_state, grid.nb_nodes, mission, robot_dir,
                                          line_detected, is_intersection))

    publish()

    try:
        while connection.step():
            current_time = connection.get_time()
            frame = connection.get_frame()
            if frame is None:  # image perdue : on ne roule pas à l'aveugle
                controller.stop(current_time)
                continue

            detection = processor.process_frame(frame)
            connection.publish_frame(processor.annotate(frame, detection))
            cx, is_intersection = detection.cx, detection.is_intersection

            if robot_state == RobotState.FOLLOWING:
                # Réarmement après avoir quitté complètement l'intersection
                if not is_intersection:
                    intersection_armed = True

                if is_intersection and intersection_armed:
                    intersection_armed = False
                    robot_state = RobotState.APPROACHING_INTERSECTION
                    action_start_time = current_time
                    approach_duration = controller.approach_duration(APPROACH_DURATION)
                    print("Intersection détectée !")

                if cx is not None:
                    half_width = detection.roi_w / 2
                    controller.aim(current_time, (cx - half_width) / half_width)
                else:
                    controller.forward(current_time)

            elif robot_state == RobotState.APPROACHING_INTERSECTION:
                controller.forward(current_time)

                if current_time - action_start_time > approach_duration:
                    if mode == Mode.HUIT:
                        robot_state = RobotState.FOLLOWING
                    else:
                        robot_state = RobotState.STOP_ON_INTERSECTION
                        action_start_time = current_time
                        print("Arrivée au centre de l'intersection.")

            elif robot_state == RobotState.STOP_ON_INTERSECTION:
                if controller.stop(current_time):
                    was_at_destination = mission.destination_reached
                    next_dir = mission.on_intersection()

                    if mission.destination_reached and not was_at_destination:
                        print("Destination atteinte ! Demi-tour, retour au point de départ.")

                    if next_dir is None:
                        print("Retour au point de départ.")
                        robot_state = RobotState.STOPPED
                    else:
                        print("Direction suivante :", next_dir, "- orientation actuelle :", robot_dir)
                        robot_state = get_turn_state(robot_dir, next_dir)
                        robot_dir = next_dir
                        action_start_time = current_time
                        print("Nouvel état :", robot_state)

            elif robot_state == RobotState.TURNING_90:
                if controller.turn(current_time, True, action_start_time):
                    print("Rotation gauche terminée.")
                    robot_state = RobotState.FOLLOWING

            elif robot_state == RobotState.TURNING_MINUS_90:
                if controller.turn(current_time, False, action_start_time):
                    print("Rotation droite terminée.")
                    robot_state = RobotState.FOLLOWING

            elif robot_state == RobotState.TURNING_180:
                if controller.turn(current_time, False, action_start_time, quarter_turns=2):
                    print("Demi-tour terminé.")
                    robot_state = RobotState.FOLLOWING

            elif robot_state == RobotState.STOPPED:
                controller.halt()
                print("Robot arrêté.")
                publish()
                break

            publish(cx is not None, is_intersection)

    finally:
        controller.halt()

    return mission
