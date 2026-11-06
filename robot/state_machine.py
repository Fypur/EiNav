"""Machine à états du robot (attente d'un ordre, livraison sur la grille, parcours en 8)."""
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
    """Attend un ordre de l'interface, moteurs coupés, en diffusant la caméra. None si le programme se termine."""
    obstacle = ObstacleGuard()
    while connection.step():
        if telemetry.take_map_reset():
            grid.clear_blocked()
            last_mission, line_lost = None, False  # plus d'arête, de chemin ni d'alerte affichés
        obstacle.update(serial.get_distance_cm())  # pour voir si le capteur marche avant de partir
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
    """Boucle principale : on exécute les ordres de l'interface jusqu'à la fin du programme."""
    processor = processor or ImageProcessor()
    last_mission: Optional[Mission] = None
    line_lost = False
    grid = Grid(GRID_SIZE)  # gardée entre les missions car un obstacle ne disparaît pas tout seul
    while True:
        request = wait_for_mission(connection, serial, telemetry, processor, grid, last_mission, line_lost)
        if request is None:
            return
        try:
            if isinstance(request, FigureEightRequest):
                last_mission = None  # pour ne plus afficher l'ancienne livraison
                line_lost = run_figure_eight(connection, serial, processor, telemetry)
            else:
                line_lost = False
                try:
                    last_mission = run_state_machine(connection, serial, request.start, request.destination,
                                                     request.heading, processor, telemetry, grid)
                    line_lost = last_mission.line_lost
                except ValueError as exc:  # plus de chemin à cause des arêtes bloquées
                    print(exc)
                    last_mission = Mission([request.start, request.destination], request.heading)
                    last_mission.no_route = True
        finally:
            telemetry.mission_finished()


class LineLossWatch:
    """Détecte si la ligne reste invisible plus de LINE_LOST_TIMEOUT (en suivi ou en recherche).

    Retient aussi de quel côté on a vu la ligne pour la dernière fois, pour chercher du bon côté.
    Le suivi se termine toujours sur une image avec la ligne, donc le compteur repart de zéro
    après un virage ou un croisement.
    now vient de Control.drive_time pour ne pas compter le temps bloqué devant un obstacle.
    """

    def __init__(self, default_left: bool = True) -> None:
        self._lost_since: Optional[float] = None
        self.last_turn_left: bool = default_left

    def update(self, detection: LineDetection) -> None:
        """Retient le côté de la ligne si on la voit."""
        if detection.cx is not None:
            half_width = detection.image_w / 2
            if detection.cx != half_width:
                self.last_turn_left = detection.cx < half_width

    def reset_timer(self) -> None:
        self._lost_since = None

    def is_lost(self, now: float, line_seen: bool) -> bool:
        if line_seen:
            self._lost_since = None
            return False
        if self._lost_since is None:
            self._lost_since = now
        return now - self._lost_since > utils.LINE_LOST_TIMEOUT


def _steer(controller: Control, detection: LineDetection, now: float) -> None:
    """Suit la ligne si on la voit, sinon tout droit doucement."""
    if detection.cx is not None:
        half_width = detection.image_w / 2
        controller.aim(now, (detection.cx - half_width) / half_width)
    else:
        controller.forward(now)


def run_figure_eight(
    connection: RobotConnection,
    serial: Serial,
    processor: ImageProcessor,
    telemetry: TelemetryServer,
) -> bool:
    """Suit la ligne et passe les croisements tout droit, jusqu'au clic sur Arrêter.

    Renvoie True si on s'est arrêté parce que la ligne est perdue.
    """
    controller = Control(serial)
    line_watch = LineLossWatch()
    obstacle = ObstacleGuard()
    robot_state = RobotState.FOLLOWING
    crossing_start = crossing_duration = 0.0
    # Le robot peut démarrer sur un croisement, il faut d'abord l'avoir quitté pour compter le suivant
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
            if frame is None:  # pas d'image, on ne roule pas à l'aveugle
                controller.stop(current_time)
                continue

            detection = processor.process_frame(frame)
            connection.publish_frame(processor.annotate(frame, detection))
            if detection.cx is not None:
                line_watch.update(detection)

            if robot_state in (RobotState.FOLLOWING, RobotState.SEARCHING_LINE):
                if detection.cx is not None:
                    robot_state = RobotState.FOLLOWING
                    line_watch.reset_timer()

                    if not detection.is_intersection:
                        intersection_armed = True
                    if detection.is_intersection and intersection_armed:
                        intersection_armed = False
                        robot_state = RobotState.APPROACHING_INTERSECTION
                        crossing_start = controller.drive_time
                        crossing_duration = controller.approach_duration(utils.APPROACH_DURATION)
                    _steer(controller, detection, current_time)
                else:
                    robot_state = RobotState.SEARCHING_LINE
                    if line_watch.is_lost(controller.drive_time, False):
                        print("Ligne perdue : arrêt du robot.")
                        return True
                    controller.spin(current_time, line_watch.last_turn_left)
            else:
                # Pas de correction ici, sinon le robot part sur l'autre branche du croisement
                controller.forward(current_time)
                if controller.drive_time - crossing_start > crossing_duration:
                    robot_state = RobotState.FOLLOWING
                    line_watch.reset_timer()

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
    """Mission de livraison : aller à end_pos, faire un tour complet, puis revenir à start_pos.

    Si un obstacle bloque la ligne, on enlève l'arête du graphe, on retourne au dernier noeud à
    vitesse réduite et on repart par un autre chemin. S'il n'y a plus de chemin, la mission s'arrête.

    Renvoie la mission avec ses flags.
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

    # Au départ le robot est sur le croisement et la caméra ne peut pas le voir, donc on part de cet état
    robot_state = RobotState.STOP_ON_INTERSECTION
    action_start_time = connection.get_time()
    approach_start = 0.0
    approach_duration = utils.APPROACH_DURATION
    delivery_half_turns = 0
    state_after_delivery = RobotState.FOLLOWING
    blocked_since: Optional[float] = None
    # Durée d'avance à l'aveugle quand le noeud est trop près pour que la caméra le voie
    blind_return = 0.0
    # Empêche de détecter plusieurs fois la même intersection (dont celle de départ, si elle est visible)
    intersection_armed = False
    seeking_after_turn = False
    slow_return = False  # retour au dernier noeud après un obstacle, à vitesse réduite
    turn_after_detour = False  # la rotation en cours suit ce retour, elle a ses propres durées

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

            speed_cap = obstacle.update(serial.get_distance_cm())
            # Pas pendant l'approche : elle doit rester à FORWARD_SPEED pour s'arrêter au centre du croisement
            if slow_return and robot_state in (RobotState.FOLLOWING, RobotState.SEARCHING_LINE):
                speed_cap = min(speed_cap, utils.DETOUR_SPEED)
            controller.set_speed_cap(speed_cap)
            current_time = connection.get_time()
            frame = connection.get_frame()
            if frame is None:  # pas d'image, on ne roule pas à l'aveugle
                controller.stop(current_time)
                continue

            detection = processor.process_frame(frame)
            connection.publish_frame(processor.annotate(frame, detection))
            cx, is_intersection = detection.cx, detection.is_intersection
            if cx is not None:
                line_watch.update(detection)

            on_edge = robot_state in (RobotState.FOLLOWING, RobotState.SEARCHING_LINE, RobotState.APPROACHING_INTERSECTION)
            if not on_edge or obstacle.status.state != ObstacleState.BLOCKED:
                blocked_since = None
            elif blocked_since is None:
                blocked_since = current_time
            elif current_time - blocked_since > utils.OBSTACLE_CONFIRM_SECONDS:
                blocked_since = None
                node, blocked_node = mission.position, mission.next_node
                if blocked_node == node:  # on revenait déjà vers ce noeud, donc obstacle des deux côtés
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
                    slow_return, turn_after_detour = True, False  # ce demi-tour-là garde la durée normale
                    line_watch.last_turn_left = False
                    action_start_time = current_time
                    # La caméra voit le croisement à partir de la distance d'approche, pas plus près
                    seen_from = utils.FORWARD_SPEED * utils.APPROACH_DURATION
                    blind_return = controller.travel / utils.FORWARD_SPEED if controller.travel < seen_from else 0.0
                    # En retour à l'aveugle on désarme la détection, sinon le noeud est compté
                    # une 2e fois quand on le quitte
                    intersection_armed = blind_return == 0.0
                else:
                    robot_state = RobotState.STOP_ON_INTERSECTION  # il n'a pas encore quitté le noeud
                publish(cx is not None, is_intersection)
                continue

            if robot_state in (RobotState.FOLLOWING, RobotState.SEARCHING_LINE):
                if seeking_after_turn:
                    if cx is not None:
                        seeking_after_turn = False
                        robot_state = RobotState.FOLLOWING
                        line_watch.reset_timer()

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
                    else:
                        robot_state = RobotState.SEARCHING_LINE
                        if line_watch.is_lost(controller.drive_time, False):
                            print("Ligne perdue : arrêt du robot.")
                            mission.line_lost = True
                            break
                        controller.spin(current_time, line_watch.last_turn_left)
                else:
                    if cx is not None:
                        robot_state = RobotState.FOLLOWING
                        line_watch.reset_timer()

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
                    else:
                        if line_watch.is_lost(controller.drive_time, False):
                            print("Ligne perdue : arrêt du robot.")
                            mission.line_lost = True
                            break
                        _steer(controller, detection, current_time)

            elif robot_state == RobotState.APPROACHING_INTERSECTION:
                controller.forward(current_time)

                if controller.drive_time - approach_start > approach_duration:
                    robot_state = RobotState.STOP_ON_INTERSECTION
                    action_start_time = current_time
                    print("Arrivée au centre de l'intersection.")

            elif robot_state == RobotState.STOP_ON_INTERSECTION:
                if controller.stop(current_time):
                    turn_after_detour, slow_return = slow_return, False
                    was_at_destination = mission.destination_reached
                    next_dir = mission.on_intersection()
                    controller.travel = 0.0

                    if mission.destination_reached and not was_at_destination:
                        print("Destination atteinte ! Demi-tour, retour au point de départ.")
                        if mission.rerouted:
                            # L'aller à l'envers repasserait par les détours, donc on recalcule
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
                        if robot_state == RobotState.TURNING_90:
                            line_watch.last_turn_left = True
                        elif robot_state in (RobotState.TURNING_MINUS_90, RobotState.TURNING_180):
                            line_watch.last_turn_left = False
                        elif robot_state == RobotState.FOLLOWING:
                            line_watch.reset_timer()
                            seeking_after_turn = False
                        if mission.destination_reached and not was_at_destination:
                            # Après le tour complet le robot est dans le même sens, il reste le demi-tour à faire
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
                        if robot_state == RobotState.TURNING_90:
                            line_watch.last_turn_left = True
                        elif robot_state in (RobotState.TURNING_MINUS_90, RobotState.TURNING_180):
                            line_watch.last_turn_left = False
                        elif robot_state == RobotState.FOLLOWING:
                            line_watch.reset_timer()
                            seeking_after_turn = False

            elif robot_state == RobotState.TURNING_90:
                if controller.turn(current_time, True, action_start_time, after_detour=turn_after_detour):
                    print("Rotation gauche terminée.")
                    robot_state = RobotState.FOLLOWING
                    line_watch.reset_timer()
                    seeking_after_turn = True

            elif robot_state == RobotState.TURNING_MINUS_90:
                if controller.turn(current_time, False, action_start_time, after_detour=turn_after_detour):
                    print("Rotation droite terminée.")
                    robot_state = RobotState.FOLLOWING
                    line_watch.reset_timer()
                    seeking_after_turn = True

            elif robot_state == RobotState.TURNING_180:
                if controller.turn(current_time, False, action_start_time, half_turn=True, after_detour=turn_after_detour):
                    print("Demi-tour terminé.")
                    robot_state = RobotState.FOLLOWING
                    line_watch.reset_timer()
                    seeking_after_turn = True
                    if blind_return > 0.0:
                        robot_state = RobotState.APPROACHING_INTERSECTION
                        approach_start = controller.drive_time
                        approach_duration = blind_return
                        blind_return = 0.0
                        seeking_after_turn = False

            elif robot_state == RobotState.STOPPED:
                controller.halt()
                print("Robot arrêté.")
                publish()
                break

            publish(cx is not None, is_intersection)

    finally:
        controller.halt()

    return mission
