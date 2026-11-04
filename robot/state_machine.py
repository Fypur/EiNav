"""Machine à états du robot : attente d'un ordre, livraison sur la grille et parcours en 8."""
from __future__ import annotations

from typing import Optional

from connections.base import RobotConnection
from connections.serialpi import Serial
from planning.direction import Direction
from planning.navigation import Grid, Mission, Navigation, Position
from planning.utils import GRID_SIZE
from robot import utils
from robot.control import Control
from robot.states import Mode, RobotState
from robot.telemetry import FigureEightRequest, Order, TelemetryServer, make_status
from vision.image_processor import ImageProcessor, LineDetection


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
    line_lost: bool = False,
) -> Optional[Order]:
    """Moteurs coupés, diffuse la caméra jusqu'à un ordre de l'interface (None : fin du programme)."""
    while connection.step():
        frame = connection.get_frame()
        line_detected = is_intersection = False
        if frame is not None:
            detection = processor.process_frame(frame)
            connection.publish_frame(processor.annotate(frame, detection))
            line_detected, is_intersection = detection.cx is not None, detection.is_intersection
        telemetry.publish(make_status(Mode.GRID, RobotState.IDLE, GRID_SIZE + 1, last_mission,
                                      line_detected, is_intersection, line_lost))
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
    """Exécute les ordres de l'interface, jusqu'à la fin du programme."""
    processor = processor or ImageProcessor()
    last_mission: Optional[Mission] = None
    line_lost = False
    while True:
        request = wait_for_mission(connection, telemetry, processor, last_mission, line_lost)
        if request is None:
            return
        try:
            if isinstance(request, FigureEightRequest):
                last_mission = None  # la livraison précédente ne doit plus être affichée
                line_lost = run_figure_eight(connection, serial, processor, telemetry)
            else:
                last_mission = run_state_machine(connection, serial, request.start, request.destination,
                                                 request.heading, processor, telemetry)
                line_lost = last_mission.line_lost
        finally:
            telemetry.mission_finished()


class LineLossWatch:
    """Signale, pendant le suivi, une ligne restée invisible plus de LINE_LOST_TIMEOUT sans interruption.

    Le suivi ne se termine que sur une image où la ligne est vue : le compteur est donc
    toujours à zéro à la reprise suivante, et un virage ou une traversée n'est jamais compté.
    """

    def __init__(self) -> None:
        self._lost_since: Optional[float] = None

    def is_lost(self, now: float, line_seen: bool) -> bool:
        if line_seen:
            self._lost_since = None
            return False
        if self._lost_since is None:
            self._lost_since = now
        return now - self._lost_since > utils.LINE_LOST_TIMEOUT


def _steer(controller: Control, detection: LineDetection, now: float) -> None:
    """Suit la ligne si elle est vue, sinon continue tout droit à vitesse réduite."""
    if detection.cx is not None:
        half_width = detection.roi_w / 2
        controller.aim(now, (detection.cx - half_width) / half_width)
    else:
        controller.forward(now)


def run_figure_eight(
    connection: RobotConnection,
    serial: Serial,
    processor: ImageProcessor,
    telemetry: TelemetryServer,
) -> bool:
    """Suit la ligne et traverse les croisements tout droit, jusqu'à l'arrêt demandé par l'interface.

    Renvoie True si le robot s'est arrêté parce qu'il ne voyait plus la ligne.
    """
    controller = Control(serial)
    line_watch = LineLossWatch()
    robot_state = RobotState.FOLLOWING
    crossing_start = crossing_duration = 0.0
    # Le robot peut être posé sur le croisement : il ne compte qu'une fois quitté.
    intersection_armed = False

    telemetry.publish(make_status(Mode.HUIT, robot_state, GRID_SIZE + 1, None))

    try:
        while connection.step():
            if telemetry.stop_requested:
                print("Arrêt demandé depuis l'interface.")
                break

            current_time = connection.get_time()
            frame = connection.get_frame()
            if frame is None:  # image perdue : on ne roule pas à l'aveugle
                controller.stop(current_time)
                continue

            detection = processor.process_frame(frame)
            connection.publish_frame(processor.annotate(frame, detection))

            if robot_state == RobotState.FOLLOWING:
                if line_watch.is_lost(current_time, detection.cx is not None):
                    print("Ligne perdue : arrêt du robot.")
                    return True

                if not detection.is_intersection:
                    intersection_armed = True
                if detection.is_intersection and intersection_armed:
                    intersection_armed = False
                    robot_state = RobotState.APPROACHING_INTERSECTION
                    crossing_start = current_time
                    crossing_duration = controller.approach_duration(utils.APPROACH_DURATION)
                _steer(controller, detection, current_time)
            else:
                # Tout droit : la correction suivrait l'autre branche du croisement.
                controller.forward(current_time)
                if current_time - crossing_start > crossing_duration:
                    robot_state = RobotState.FOLLOWING

            telemetry.publish(make_status(Mode.HUIT, robot_state, GRID_SIZE + 1, None,
                                          detection.cx is not None, detection.is_intersection))
    finally:
        controller.halt()
    return False


def run_state_machine(
    connection: RobotConnection,
    serial: Serial,
    start_pos: Position,
    end_pos: Position,
    robot_dir: Direction,
    processor: Optional[ImageProcessor] = None,
    telemetry: Optional[TelemetryServer] = None,
) -> Mission:
    """Va de `start_pos` à `end_pos`, fait demi-tour et revient, sauf arrêt demandé par l'interface.

    Renvoie la mission (et ses flags).
    """
    controller = Control(serial)
    line_watch = LineLossWatch()
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

    # Posé sur le croisement de départ, le robot ne peut pas le voir : il y est considéré comme arrêté.
    robot_state = RobotState.STOP_ON_INTERSECTION
    action_start_time = connection.get_time()
    approach_duration = utils.APPROACH_DURATION
    # Empêche de détecter plusieurs fois la même intersection (dont celle de départ, si elle est visible)
    intersection_armed = False

    def publish(line_detected: bool = False, is_intersection: bool = False) -> None:
        if telemetry is not None:
            telemetry.publish(make_status(Mode.GRID, robot_state, grid.nb_nodes, mission,
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
                if line_watch.is_lost(current_time, cx is not None):
                    print("Ligne perdue : arrêt du robot.")
                    mission.line_lost = True
                    break

                # Réarmement après avoir quitté complètement l'intersection
                if not is_intersection:
                    intersection_armed = True

                if is_intersection and intersection_armed:
                    intersection_armed = False
                    robot_state = RobotState.APPROACHING_INTERSECTION
                    action_start_time = current_time
                    approach_duration = controller.approach_duration(utils.APPROACH_DURATION)
                    print("Intersection détectée !")

                _steer(controller, detection, current_time)

            elif robot_state == RobotState.APPROACHING_INTERSECTION:
                controller.forward(current_time)

                if current_time - action_start_time > approach_duration:
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
                if controller.turn(current_time, False, action_start_time, half_turn=True):
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
