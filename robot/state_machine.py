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
from robot.obstacle import ObstacleGuard, ObstacleState
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
    serial: Serial,
    telemetry: TelemetryServer,
    processor: ImageProcessor,
    grid: Grid,
    last_mission: Optional[Mission] = None,
    line_lost: bool = False,
) -> Optional[Order]:
    """Moteurs coupés, diffuse la caméra jusqu'à un ordre de l'interface (None : fin du programme)."""
    obstacle = ObstacleGuard()
    while connection.step():
        if telemetry.take_map_reset():
            grid.clear_blocked()
            last_mission, line_lost = None, False  # plus d'arête, de chemin ni d'alerte affichés
        obstacle.update(serial.get_distance_cm())  # pour vérifier le capteur avant de démarrer
        frame = connection.get_frame()
        line_detected = is_intersection = False
        if frame is not None:
            detection = processor.process_frame(frame)
            connection.publish_frame(processor.annotate(frame, detection))
            line_detected, is_intersection = detection.cx is not None, detection.is_intersection
        telemetry.publish(make_status(Mode.GRID, RobotState.IDLE, GRID_SIZE + 1, last_mission,
                                      line_detected, is_intersection, line_lost, obstacle.status,
                                      grid.blocked_edges))
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
    grid = Grid(GRID_SIZE)  # partagé entre les missions : un obstacle reste là d'une mission à l'autre
    while True:
        request = wait_for_mission(connection, serial, telemetry, processor, grid, last_mission, line_lost)
        if request is None:
            return
        try:
            if isinstance(request, FigureEightRequest):
                last_mission = None  # la livraison précédente ne doit plus être affichée
                line_lost = run_figure_eight(connection, serial, processor, telemetry)
            else:
                line_lost = False
                try:
                    last_mission = run_state_machine(connection, serial, request.start, request.destination,
                                                     request.heading, processor, telemetry, grid)
                    line_lost = last_mission.line_lost
                except ValueError as exc:  # destination coupée par des arêtes bloquées
                    print(exc)
                    last_mission = Mission([request.start, request.destination], request.heading)
                    last_mission.no_route = True
        finally:
            telemetry.mission_finished()


class LineLossWatch:
    """Signale, pendant le suivi, une ligne restée invisible plus de LINE_LOST_TIMEOUT sans interruption.

    Le suivi ne se termine que sur une image où la ligne est vue : le compteur est donc
    toujours à zéro à la reprise suivante, et un virage ou une traversée n'est jamais compté.
    `now` est `Control.drive_time` : le temps passé bloqué devant un obstacle ne compte pas non plus.
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
    obstacle = ObstacleGuard()
    robot_state = RobotState.FOLLOWING
    crossing_start = crossing_duration = 0.0
    # Le robot peut être posé sur le croisement : il ne compte qu'une fois quitté.
    intersection_armed = False

    controller.set_speed_cap(obstacle.update(serial.get_distance_cm()))
    telemetry.publish(make_status(Mode.HUIT, robot_state, GRID_SIZE + 1, None, obstacle=obstacle.status))

    try:
        while connection.step():
            if telemetry.stop_requested:
                print("Arrêt demandé depuis l'interface.")
                break

            controller.set_speed_cap(obstacle.update(serial.get_distance_cm()))
            current_time = connection.get_time()
            frame = connection.get_frame()
            if frame is None:  # image perdue : on ne roule pas à l'aveugle
                controller.stop(current_time)
                continue

            detection = processor.process_frame(frame)
            connection.publish_frame(processor.annotate(frame, detection))

            if robot_state == RobotState.FOLLOWING:
                if line_watch.is_lost(controller.drive_time, detection.cx is not None):
                    print("Ligne perdue : arrêt du robot.")
                    return True

                if not detection.is_intersection:
                    intersection_armed = True
                if detection.is_intersection and intersection_armed:
                    intersection_armed = False
                    robot_state = RobotState.APPROACHING_INTERSECTION
                    crossing_start = controller.drive_time
                    crossing_duration = controller.approach_duration(utils.APPROACH_DURATION)
                _steer(controller, detection, current_time)
            else:
                # Tout droit : la correction suivrait l'autre branche du croisement.
                controller.forward(current_time)
                if controller.drive_time - crossing_start > crossing_duration:
                    robot_state = RobotState.FOLLOWING

            telemetry.publish(make_status(Mode.HUIT, robot_state, GRID_SIZE + 1, None,
                                          detection.cx is not None, detection.is_intersection,
                                          obstacle=obstacle.status))
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
    grid: Optional[Grid] = None,
) -> Mission:
    """Va de `start_pos` à `end_pos`, y fait un tour complet (la livraison), puis fait demi-tour
    et revient, sauf arrêt demandé par l'interface.

    Arrêté par un obstacle sur une ligne, le robot retire l'arête du graphe, revient à son dernier
    nœud et repart par un nouveau chemin. La mission s'arrête s'il n'y en a aucun.

    Renvoie la mission (et ses flags).
    """
    controller = Control(serial)
    line_watch = LineLossWatch()
    obstacle = ObstacleGuard()
    processor = processor or ImageProcessor()

    grid = grid or Grid(GRID_SIZE)
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
    approach_start = 0.0
    approach_duration = utils.APPROACH_DURATION
    delivery_half_turns = 0
    state_after_delivery = RobotState.FOLLOWING
    blocked_since: Optional[float] = None
    # Retour à un nœud trop proche pour que la caméra le voie : durée d'avance à l'aveugle.
    blind_return = 0.0
    # Empêche de détecter plusieurs fois la même intersection (dont celle de départ, si elle est visible)
    intersection_armed = False

    def publish(line_detected: bool = False, is_intersection: bool = False) -> None:
        if telemetry is not None:
            telemetry.publish(make_status(Mode.GRID, robot_state, grid.nb_nodes, mission,
                                          line_detected, is_intersection, obstacle=obstacle.status,
                                          blocked_edges=grid.blocked_edges))

    controller.set_speed_cap(obstacle.update(serial.get_distance_cm()))
    publish()

    try:
        while connection.step():
            if telemetry is not None and telemetry.stop_requested:
                print("Arrêt demandé depuis l'interface.")
                break

            controller.set_speed_cap(obstacle.update(serial.get_distance_cm()))
            current_time = connection.get_time()
            frame = connection.get_frame()
            if frame is None:  # image perdue : on ne roule pas à l'aveugle
                controller.stop(current_time)
                continue

            detection = processor.process_frame(frame)
            connection.publish_frame(processor.annotate(frame, detection))
            cx, is_intersection = detection.cx, detection.is_intersection

            on_edge = robot_state in (RobotState.FOLLOWING, RobotState.APPROACHING_INTERSECTION)
            if not on_edge or obstacle.status.state != ObstacleState.BLOCKED:
                blocked_since = None
            elif blocked_since is None:
                blocked_since = current_time
            elif current_time - blocked_since > utils.OBSTACLE_CONFIRM_SECONDS:
                blocked_since = None
                node, blocked_node = mission.position, mission.next_node
                if blocked_node == node:  # déjà en retour vers ce nœud : obstacle des deux côtés
                    print("Obstacle sur le retour au nœud : mission arrêtée.")
                    mission.no_route = True
                    break
                grid.block_edge(node, blocked_node)
                print(f"Arête bloquée : {node} - {blocked_node}")
                moved = controller.travel > 0.0
                path = nav.generate_path(grid, node, mission.target, robot_dir.opposite if moved else robot_dir)
                if not path:
                    print("Aucun chemin : mission arrêtée.")
                    mission.no_route = True
                    break
                print("Nouveau chemin :", path)
                nav.display_path(grid, path)
                mission.reroute(path)
                if moved:
                    robot_dir = robot_dir.opposite
                    mission.heading = robot_dir
                    robot_state = RobotState.TURNING_180
                    action_start_time = current_time
                    # La caméra voit à la distance parcourue pendant l'approche, pas plus près.
                    seen_from = utils.FORWARD_SPEED * utils.APPROACH_DURATION
                    blind_return = controller.travel / utils.FORWARD_SPEED if controller.travel < seen_from else 0.0
                    # Le nœud doit pouvoir être détecté sur le retour, sauf si on y retourne à l'aveugle :
                    # il serait alors compté une seconde fois en le quittant.
                    intersection_armed = blind_return == 0.0
                else:
                    robot_state = RobotState.STOP_ON_INTERSECTION  # n'a pas quitté le nœud
                publish(cx is not None, is_intersection)
                continue

            if robot_state == RobotState.FOLLOWING:
                if line_watch.is_lost(controller.drive_time, cx is not None):
                    print("Ligne perdue : arrêt du robot.")
                    mission.line_lost = True
                    break

                # Réarmement après avoir quitté complètement l'intersection
                if not is_intersection:
                    intersection_armed = True

                if is_intersection and intersection_armed:
                    intersection_armed = False
                    robot_state = RobotState.APPROACHING_INTERSECTION
                    approach_start = controller.drive_time
                    approach_duration = controller.approach_duration(utils.APPROACH_DURATION)
                    print("Intersection détectée !")

                _steer(controller, detection, current_time)

            elif robot_state == RobotState.APPROACHING_INTERSECTION:
                controller.forward(current_time)

                if controller.drive_time - approach_start > approach_duration:
                    robot_state = RobotState.STOP_ON_INTERSECTION
                    action_start_time = current_time
                    print("Arrivée au centre de l'intersection.")

            elif robot_state == RobotState.STOP_ON_INTERSECTION:
                if controller.stop(current_time):
                    was_at_destination = mission.destination_reached
                    next_dir = mission.on_intersection()
                    controller.travel = 0.0

                    if mission.destination_reached and not was_at_destination:
                        print("Destination atteinte ! Demi-tour, retour au point de départ.")
                        if mission.rerouted:
                            # Refaire l'aller à l'envers repasserait par les détours : on recalcule.
                            path = nav.generate_path(grid, mission.destination, mission.start, robot_dir)
                            if not path:
                                print("Aucun chemin de retour : mission arrêtée.")
                                mission.no_route = True
                                break
                            mission.reroute(path)
                            next_dir = mission.on_intersection()

                    if next_dir is None:
                        print("Retour au point de départ.")
                        robot_state = RobotState.STOPPED
                    else:
                        print("Direction suivante :", next_dir, "- orientation actuelle :", robot_dir)
                        robot_state = get_turn_state(robot_dir, next_dir)
                        robot_dir = next_dir
                        mission.heading = next_dir
                        action_start_time = current_time
                        if mission.destination_reached and not was_at_destination:
                            # Le tour complet ramène le robot dans le sens d'arrivée : le demi-tour reste à faire.
                            state_after_delivery = robot_state
                            robot_state = RobotState.DELIVERING
                            delivery_half_turns = utils.DELIVERY_HALF_TURNS
                        print("Nouvel état :", robot_state)

            elif robot_state == RobotState.DELIVERING:
                if controller.turn(current_time, False, action_start_time, half_turn=True):
                    delivery_half_turns -= 1
                    action_start_time = current_time
                    if delivery_half_turns == 0:
                        print("Livraison terminée.")
                        robot_state = state_after_delivery

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
                    if blind_return > 0.0:
                        robot_state = RobotState.APPROACHING_INTERSECTION
                        approach_start = controller.drive_time
                        approach_duration = blind_return
                        blind_return = 0.0

            elif robot_state == RobotState.STOPPED:
                controller.halt()
                print("Robot arrêté.")
                publish()
                break

            publish(cx is not None, is_intersection)

    finally:
        controller.halt()

    return mission
