"""Machine à états du robot : attente d'un ordre, suivi de ligne et navigation sur la grille."""
from __future__ import annotations

from typing import Optional

from connections.base import RobotConnection
from connections.serialpi import Serial
from planning.direction import Direction
from planning.navigation import GRID_SIZE, Grid, Mission, Navigation, Position
from robot.control import Control
from robot.states import Mode, RobotState
from robot.telemetry import MissionRequest, TelemetryServer, make_status
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


def wait_for_mission(
    connection: RobotConnection,
    telemetry: TelemetryServer,
    processor: ImageProcessor,
    last_mission: Optional[Mission] = None,
) -> Optional[MissionRequest]:
    """Moteurs coupés, diffuse la caméra jusqu'à un ordre de l'interface (None : fin du programme)."""
    while connection.step():
        frame = connection.get_frame()
        line_detected = is_intersection = False
        if frame is not None:
            detection = processor.process_frame(frame)
            connection.publish_frame(processor.annotate(frame, detection))
            line_detected, is_intersection = detection.cx is not None, detection.is_intersection
        telemetry.publish(make_status(Mode.GRID, RobotState.IDLE, GRID_SIZE + 1, last_mission,
                                      line_detected, is_intersection))
        request = telemetry.take_mission()
        if request is not None:
            return request
    return None


def serve_missions(
    connection: RobotConnection,
    serial: Serial,
    telemetry: TelemetryServer,
    processor: Optional[ImageProcessor] = None,
) -> None:
    """Exécute les livraisons demandées par l'interface, jusqu'à la fin du programme."""
    processor = processor or ImageProcessor()
    last_mission: Optional[Mission] = None
    while True:
        request = wait_for_mission(connection, telemetry, processor, last_mission)
        if request is None:
            return
        try:
            last_mission = run_state_machine(connection, serial, request.start, request.destination,
                                             request.heading, Mode.GRID, processor, telemetry)
        finally:
            telemetry.mission_finished()


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
    """Va de `start_pos` à `end_pos`, fait demi-tour et revient, sauf arrêt demandé par l'interface.

    Renvoie la mission (et ses flags).
    """
    controller = Control(serial)
    processor = processor or ImageProcessor()

    grid = Grid(GRID_SIZE)
    nav = Navigation()
    path = nav.generate_path(grid, start_pos, end_pos, robot_dir)
    if not path:
        controller.halt()
        raise ValueError(f"Impossible de calculer le chemin de {start_pos} à {end_pos}.")

    print("Chemin :", path)
    nav.display_path(grid, path)
    mission = Mission(path, robot_dir)

    robot_state = RobotState.FOLLOWING
    action_start_time = connection.get_time()
    approach_duration = APPROACH_DURATION
    # Empêche de détecter plusieurs fois la même intersection
    intersection_armed = True

    def publish(line_detected: bool = False, is_intersection: bool = False) -> None:
        if telemetry is not None:
            telemetry.publish(make_status(mode, robot_state, grid.nb_nodes, mission,
                                          line_detected, is_intersection))

    publish()

    try:
        while connection.step():
            if telemetry is not None and telemetry.stop_requested:
                print("Arrêt demandé depuis l'interface.")
                break

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
                        mission.heading = next_dir
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
