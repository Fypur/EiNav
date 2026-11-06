"""Tests de l'arête bloquée par un obstacle (on l'enlève du graphe et on recalcule le chemin), avec un robot simulé."""
from __future__ import annotations

import json
import math
import threading
import time
import urllib.error
import urllib.request
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np
import pytest

from connections.base import RobotConnection
from planning.direction import Direction
from planning.navigation import Grid, Mission, Navigation
from robot import tuning, utils
from robot.state_machine import run_state_machine, serve_missions
from robot.states import RobotState
from robot.telemetry import TelemetryServer
from vision.image_processor import ImageProcessor, LineDetection

SPACING = 60.0  # cm entre deux croisements
GAIN = 80.0  # cm/s à pleine vitesse
ANGLE = {Direction.NORTH: 0.0, Direction.EAST: 90.0, Direction.SOUTH: 180.0, Direction.WEST: 270.0}


# Graphe

def test_arete_bloquee_dans_les_deux_sens() -> None:
    grid = Grid()
    grid.block_edge((1, 1), (1, 2))
    assert grid.grid[1][1].bottom is False and grid.grid[2][1].top is False
    assert grid.grid[1][1].right is True, "les autres arêtes du nœud restent ouvertes"
    assert grid.blocked_edges == [((1, 1), (1, 2))]


def test_arete_identique_quel_que_soit_le_sens_et_sans_doublon() -> None:
    grid = Grid()
    grid.block_edge((1, 2), (1, 1))
    grid.block_edge((1, 1), (1, 2))
    assert grid.blocked_edges == [((1, 1), (1, 2))]


def test_chemin_evite_l_arete_bloquee() -> None:
    grid = Grid()
    assert Navigation().generate_path(grid, (0, 0), (0, 2), Direction.SOUTH) == [(0, 0), (0, 1), (0, 2)]
    grid.block_edge((0, 1), (0, 2))
    path = Navigation().generate_path(grid, (0, 1), (0, 2), Direction.NORTH)
    assert path[0] == (0, 1) and path[-1] == (0, 2)
    edges = {frozenset(pair) for pair in zip(path, path[1:])}
    assert frozenset(((0, 1), (0, 2))) not in edges
    assert len(path) == 4, "le détour le plus court"


def test_aucun_chemin_quand_la_destination_est_isolee() -> None:
    grid = Grid()
    grid.block_edge((0, 0), (1, 0))
    grid.block_edge((0, 0), (0, 1))
    assert Navigation().generate_path(grid, (2, 2), (0, 0), Direction.NORTH) == []


def test_deblocage_ne_rouvre_pas_les_bords_de_la_grille() -> None:
    grid = Grid()
    grid.block_edge((0, 0), (1, 0))
    grid.clear_blocked()
    assert grid.blocked_edges == []
    assert grid.grid[0][0].right is True
    assert grid.grid[0][0].left is False and grid.grid[0][0].top is False


@pytest.mark.parametrize("direction,opposite", [(Direction.NORTH, Direction.SOUTH), (Direction.EAST, Direction.WEST)])
def test_direction_opposee(direction: Direction, opposite: Direction) -> None:
    assert direction.opposite == opposite and opposite.opposite == direction


def test_mission_reroutee_repart_du_nœud_courant() -> None:
    mission = Mission([(0, 0), (0, 1), (0, 2)], Direction.SOUTH)
    mission.on_intersection()
    mission.on_intersection()
    assert mission.position == (0, 1) and mission.next_node == (0, 2)
    mission.reroute([(0, 1), (1, 1), (1, 2), (0, 2)])
    assert mission.position == (0, 1) and mission.next_node == (0, 1), "d'abord retourner au nœud"
    assert mission.target == (0, 2)
    assert mission.on_intersection() == Direction.EAST
    assert mission.next_node == (1, 1)
    assert mission.destination == (0, 2) and mission.start == (0, 0)


# Monde simulé

class World(RobotConnection, ImageProcessor):
    """Fausse grille où le robot avance selon ses vitesses de roue et voit les croisements et les obstacles."""

    def __init__(self, start: Tuple[int, int], heading: Direction, obstacles: Sequence[Tuple[float, float]] = (),
                 max_steps: int = 6000, camera_nearest: float = 2.0) -> None:
        self.camera_nearest = camera_nearest  # plus près que ça le croisement est sous le robot, donc invisible
        self.x, self.y = start[0] * SPACING, start[1] * SPACING
        self.angle = ANGLE[heading]
        self.obstacles = list(obstacles)
        self.dt = 0.064
        self.t = 0.0
        self.steps = 0
        self.max_steps = max_steps
        self.left = self.right = 0.0
        self.closest = math.inf
        self.deg_per_second = 180.0 / (utils.TURN_SPEED * utils.TURN_180_DURATION)
        # Caméra calibrée : on voit le croisement à la distance parcourue pendant l'approche
        self.camera_reach = utils.FORWARD_SPEED * utils.APPROACH_DURATION * GAIN

    # Robot
    def get_time(self) -> float:
        return self.t

    def step(self) -> bool:
        self.t += self.dt
        self.steps += 1
        linear, angular = (self.left + self.right) / 2, (self.left - self.right) / 2
        self.angle = (self.angle + angular * self.deg_per_second * self.dt) % 360
        if abs(angular) < 1e-9 and linear > 0:  # le suivi de ligne remet le robot sur la ligne
            self.angle = round(self.angle / 90) * 90 % 360
            if self.angle % 180 == 0:
                self.x = round(self.x / SPACING) * SPACING
            else:
                self.y = round(self.y / SPACING) * SPACING
        dx, dy = self._forward()
        self.x += linear * GAIN * self.dt * dx
        self.y += linear * GAIN * self.dt * dy
        for ox, oy in self.obstacles:
            self.closest = min(self.closest, math.hypot(ox * SPACING - self.x, oy * SPACING - self.y))
        return self.steps < self.max_steps

    def _forward(self) -> Tuple[float, float]:
        radians = math.radians(self.angle)
        return math.sin(radians), -math.cos(radians)

    def get_frame(self) -> np.ndarray:
        return np.zeros((128, 160, 3), np.uint8)

    def publish_frame(self, frame: np.ndarray) -> None:
        pass

    # Liaison série
    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        self.left, self.right = fl, fr

    def get_distance_cm(self) -> Optional[int]:
        dx, dy = self._forward()
        nearest = 200.0
        for ox, oy in self.obstacles:
            rx, ry = ox * SPACING - self.x, oy * SPACING - self.y
            distance = math.hypot(rx, ry)
            if distance > 0 and (rx * dx + ry * dy) / distance >= math.cos(math.radians(30)):
                nearest = min(nearest, distance)
        return int(nearest)

    # Caméra
    def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
        dx, dy = self._forward()
        seen = False
        for nx in range(5):
            for ny in range(5):
                rx, ry = nx * SPACING - self.x, ny * SPACING - self.y
                along, side = rx * dx + ry * dy, abs(rx * dy - ry * dx)
                # La caméra regarde devant, elle ne voit pas le croisement sous le robot
                seen = seen or (side < 6.0 and max(self.camera_nearest, self.camera_reach - 8.0) <= along <= self.camera_reach)
        return LineDetection(80, seen, 160, 76, np.zeros((52, 160), np.uint8))

    def annotate(self, image_bgr: np.ndarray, detection: LineDetection) -> np.ndarray:
        return image_bgr

    def at(self, node: Tuple[int, int]) -> float:
        return math.hypot(node[0] * SPACING - self.x, node[1] * SPACING - self.y)


class Recorder:
    stop_requested = False

    def __init__(self) -> None:
        self.statuses: List[Any] = []

    def publish(self, status: Any) -> None:
        self.statuses.append(status)

    def nodes(self) -> List[Tuple[int, int]]:
        visited: List[Tuple[int, int]] = []
        for status in self.statuses:
            if status.position is not None and (not visited or visited[-1] != status.position):
                visited.append(status.position)
        return visited


class Watch(Recorder):
    """Mesure l'écart au noeud quand le robot en repart après le blocage de l'arête."""

    def __init__(self, world: World, node: Tuple[int, int]) -> None:
        super().__init__()
        self.world = world
        self.node = node
        self.offset: Optional[float] = None
        self.previous = ""

    def publish(self, status: Any) -> None:
        super().publish(status)
        if self.previous == RobotState.STOP_ON_INTERSECTION.name and status.state != self.previous                 and status.position == self.node and status.blocked_edges and self.offset is None:
            self.offset = self.world.at(self.node)
        self.previous = status.state

    def state_after_u_turn(self) -> str:
        """État juste après le demi-tour de retour au noeud."""
        states = [s.state for s in self.statuses if s.blocked_edges]
        assert states[0] == RobotState.TURNING_180.name
        return next(state for state in states
                    if state not in (RobotState.TURNING_180.name, RobotState.ALIGNING_LINE.name))


def _watched(world: World, node: Tuple[int, int]) -> Tuple[Mission, Watch, Grid]:
    watch, grid = Watch(world, node), Grid()
    mission = run_state_machine(world, world, (0, 0), (0, 2), Direction.SOUTH, world, watch,  # type: ignore[arg-type]
                                grid=grid)
    return mission, watch, grid


def _run(world: World, start: Tuple[int, int], end: Tuple[int, int], heading: Direction,
         grid: Optional[Grid] = None) -> Tuple[Mission, Recorder, Grid]:
    recorder = Recorder()
    grid = grid or Grid()
    mission = run_state_machine(world, world, start, end, heading, world, recorder,  # type: ignore[arg-type]
                                grid=grid)
    return mission, recorder, grid


def _edges(nodes: Sequence[Tuple[int, int]]) -> set:
    return {frozenset(pair) for pair in zip(nodes, nodes[1:])}


# Retour à vitesse réduite après un obstacle

class SpeedLog(Recorder):
    """Note à chaque pas l'état, la vitesse d'avance envoyée aux roues et l'heure."""

    def __init__(self, world: World) -> None:
        super().__init__()
        self.world = world
        self.log: List[Tuple[str, float, float, bool]] = []

    def publish(self, status: Any) -> None:
        super().publish(status)
        speed = (self.world.left + self.world.right) / 2
        self.log.append((status.state, speed, self.world.t, bool(status.blocked_edges)))

    def speeds(self, state: str, blocked: bool) -> List[float]:
        return [speed for name, speed, _, cut in self.log if name == state and cut == blocked]

    def durations(self, state: RobotState) -> List[float]:
        """Durée de chaque passage dans cet état après le blocage de l'arête, en secondes."""
        result: List[float] = []
        previous, started = "", 0.0
        for name, _, t, cut in self.log:
            if not cut:
                continue
            if name != previous:
                if previous == state.name:
                    result.append(t - started)
                previous, started = name, t
        return result


def _logged(start: Tuple[int, int], end: Tuple[int, int], obstacle: Tuple[float, float],
            **settings: float) -> Tuple[Mission, SpeedLog]:
    tuning.reset()
    tuning.update(settings)
    world = World(start, Direction.SOUTH, obstacles=[obstacle])
    log = SpeedLog(world)
    mission = run_state_machine(world, world, start, end, Direction.SOUTH, world, log,  # type: ignore[arg-type]
                                grid=Grid())
    assert mission.destination_reached and mission.returned_home
    return mission, log


APPROACHING = RobotState.APPROACHING_INTERSECTION


def _segment_after_u_turn(log: SpeedLog) -> List[Tuple[str, float]]:
    """(état, vitesse) entre la fin du demi-tour et l'arrivée au noeud."""
    states = [(name, speed) for name, speed, _, cut in log.log if cut]
    start = next(i for i, (name, _) in enumerate(states) if name != RobotState.TURNING_180.name)
    end = next(i for i in range(start, len(states)) if states[i][0] == RobotState.STOP_ON_INTERSECTION.name)
    return states[start:end]


def test_retour_au_noeud_a_vitesse_reduite_reglable() -> None:
    for speed in (0.15, 0.3):
        # Obstacle loin du noeud : le robot a la place d'atteindre sa vitesse sur le retour
        _, log = _logged((0, 0), (0, 3), (0, 1.9), DETOUR_SPEED=speed)
        following = [v for name, v in _segment_after_u_turn(log) if name == RobotState.FOLLOWING.name]
        assert following and max(following) == pytest.approx(speed, abs=0.01)


def test_avant_l_obstacle_et_apres_le_noeud_vitesse_normale() -> None:
    _, log = _logged((0, 0), (0, 3), (0, 1.9), DETOUR_SPEED=0.15)
    assert max(log.speeds(RobotState.FOLLOWING.name, False)) == pytest.approx(utils.BASE_SPEED, abs=0.01)
    on_return = sum(name == RobotState.FOLLOWING.name for name, _ in _segment_after_u_turn(log))
    after_node = log.speeds(RobotState.FOLLOWING.name, True)[on_return:]
    assert max(after_node) == pytest.approx(utils.BASE_SPEED, abs=0.01)


def test_approche_du_noeud_garde_la_vitesse_d_approche() -> None:
    """Même avec une vitesse réduite plus basse : c'est elle qui amène au centre du croisement."""
    world = World((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)])
    tuning.update({"DETOUR_SPEED": 0.05})
    mission, watch, _ = _watched(world, (0, 1))
    assert mission.returned_home
    assert watch.offset is not None and watch.offset < 3.0, "tourne au centre du nœud"


def test_le_freinage_devant_un_obstacle_reste_prioritaire() -> None:
    """Un obstacle sur le retour arrête le robot, même à vitesse réduite."""
    class ObstacleBehind(World):
        def step(self) -> bool:
            if self.obstacles == [(0, 1.5)] and self.angle == 0 and self.y < 1.3 * SPACING:
                self.obstacles.append((0, 1.1))
            return super().step()

    tuning.update({"DETOUR_SPEED": 0.3})
    world = ObstacleBehind((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)])
    mission, recorder, _ = _run(world, (0, 0), (0, 2), Direction.SOUTH)
    assert mission.no_route
    assert RobotState.TURNING_180.name in [s.state for s in recorder.statuses]


FAR = ((0, 0), (0, 3), (0, 1.9))  # obstacle loin du noeud : retour avec la caméra, à vitesse stabilisée


def test_approche_apres_obstacle_utilise_son_propre_reglage() -> None:
    _, short = _logged(*FAR, DETOUR_APPROACH_DURATION=0.6)
    _, long = _logged(*FAR, DETOUR_APPROACH_DURATION=1.2)
    short_times, long_times = short.durations(APPROACHING), long.durations(APPROACHING)
    assert long_times[0] - short_times[0] == pytest.approx(0.6, abs=0.15)
    assert short_times[1] == pytest.approx(long_times[1], abs=0.15), "l'approche suivante revient à la durée normale"


def test_approche_apres_obstacle_tient_compte_de_la_rampe() -> None:
    """Le robot arrive à DETOUR_SPEED : il freine moins vite que FORWARD_SPEED, comme une approche normale."""
    _, log = _logged(*FAR, DETOUR_SPEED=0.2, DETOUR_APPROACH_DURATION=1.2)
    ramp = 0.2 * utils.RAMP_TIME_CONSTANT / utils.FORWARD_SPEED
    assert log.durations(APPROACHING)[0] == pytest.approx(1.2 - ramp, abs=0.15)


def test_approche_apres_obstacle_ignore_la_duree_normale() -> None:
    _, normal = _logged(*FAR, APPROACH_DURATION=0.6)
    _, other = _logged(*FAR, APPROACH_DURATION=1.2)
    assert normal.durations(APPROACHING)[0] == pytest.approx(other.durations(APPROACHING)[0], abs=0.15)
    assert other.durations(APPROACHING)[1] - normal.durations(APPROACHING)[1] == pytest.approx(0.6, abs=0.15)


def test_approche_apres_obstacle_change_l_arret_sur_le_noeud() -> None:
    """Le réglage déplace vraiment le point d'arrêt du robot sur le croisement."""
    def stop_offset(duration: float) -> float:
        tuning.update({"DETOUR_APPROACH_DURATION": duration})
        world = World((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)])
        _, watch, _ = _watched(world, (0, 1))
        assert watch.offset is not None
        return watch.offset

    assert stop_offset(utils.APPROACH_DURATION) < 3.0, "par défaut, au centre"
    assert stop_offset(utils.APPROACH_DURATION + 1.0) > 5.0, "plus longtemps, il dépasse le centre"


def test_retour_a_l_aveugle_ne_depend_pas_du_reglage() -> None:
    """Pas de détection dans ce cas : le robot refait la distance parcourue."""
    near = (0, 1 + 24.0 / SPACING)
    _, short = _logged((0, 0), (0, 2), near, DETOUR_APPROACH_DURATION=0.3)
    _, long = _logged((0, 0), (0, 2), near, DETOUR_APPROACH_DURATION=1.5)
    assert short.durations(APPROACHING)[0] == pytest.approx(long.durations(APPROACHING)[0], abs=0.15)


def test_sans_retour_la_duree_normale_s_applique() -> None:
    """Obstacle vu depuis le noeud de départ : le robot n'a pas bougé, donc pas de parcours à vitesse réduite."""
    obstacle = (0, 18.0 / SPACING)
    _, short = _logged((0, 0), (0, 2), obstacle, DETOUR_APPROACH_DURATION=0.3)
    _, long = _logged((0, 0), (0, 2), obstacle, DETOUR_APPROACH_DURATION=1.5)
    assert short.durations(APPROACHING)[0] == pytest.approx(long.durations(APPROACHING)[0], abs=0.15)


def test_rotations_apres_le_retour_gardent_les_durees_normales() -> None:
    _, normal = _logged((0, 0), (0, 2), (0, 1.5), TURN_90_DURATION=1.0)
    _, other = _logged((0, 0), (0, 2), (0, 1.5), TURN_90_DURATION=1.3)
    first, second = normal.durations(RobotState.TURNING_MINUS_90)[0], other.durations(RobotState.TURNING_MINUS_90)[0]
    assert second - first == pytest.approx(0.3, abs=0.15)


# Alignement sur la ligne à la fin du demi-tour

class Scripted(World):
    """Le centre de la ligne vu par la caméra est donné par le test, le temps de l'alignement."""

    def __init__(self, script: Any, **kwargs: Any) -> None:
        super().__init__((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)], **kwargs)
        self.script = script
        self.aligning = False
        self.spins: List[float] = []

    def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
        detection = super().process_frame(image_bgr)
        return detection._replace(cx=self.script(self)) if self.aligning else detection

    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        super().send_wheel_speeds(fl, fr, rr, rl)
        if self.aligning:
            self.spins.append((fl - fr) / 2)


class AlignWatch(Recorder):
    def __init__(self, world: Scripted) -> None:
        super().__init__()
        self.world = world

    def publish(self, status: Any) -> None:
        super().publish(status)
        self.world.aligning = status.state == RobotState.ALIGNING_LINE.name

    def states(self) -> List[str]:
        return [s.state for s in self.statuses]


def _align(script: Any, max_steps: int = 6000) -> Tuple[Mission, AlignWatch, Scripted]:
    world = Scripted(script, max_steps=max_steps)
    watch = AlignWatch(world)
    mission = run_state_machine(world, world, (0, 0), (0, 2), Direction.SOUTH, world, watch,  # type: ignore[arg-type]
                                grid=Grid())
    return mission, watch, world


def test_demi_tour_devant_un_obstacle_passe_par_l_alignement() -> None:
    mission, watch, _ = _align(lambda world: 80)
    states = watch.states()
    assert states[states.index(RobotState.TURNING_180.name) + 1:].index(RobotState.ALIGNING_LINE.name) >= 0
    assert mission.destination_reached and mission.returned_home


def test_demi_tour_a_destination_sans_alignement() -> None:
    """Seul le demi-tour devant un obstacle est concerné, pas celui après la livraison."""
    world = World((0, 0), Direction.SOUTH)
    _, recorder, _ = _run(world, (0, 0), (0, 2), Direction.SOUTH)
    assert RobotState.ALIGNING_LINE.name not in [s.state for s in recorder.statuses]


@pytest.mark.parametrize("cx,sign", [(20, -1), (140, 1)])
def test_pivote_du_cote_de_la_ligne(cx: int, sign: int) -> None:
    """Ligne à gauche : rotation à gauche (< 0). Ligne à droite : rotation à droite (> 0)."""
    calls = []

    def script(world: World) -> int:
        calls.append(1)
        return 80 if len(calls) > 10 else cx

    _, _, world = _align(script)
    moving = [speed for speed in world.spins[:8] if abs(speed) > 0.01]
    assert moving and all(speed * sign > 0 for speed in moving)
    assert max(abs(speed) for speed in world.spins) <= utils.ALIGN_SPEED + 0.01


def test_ligne_invisible_cherche_a_droite_puis_a_gauche() -> None:
    calls = []

    def script(world: World) -> Optional[int]:
        calls.append(1)
        return None if len(calls) < 70 else 80

    mission, _, world = _align(script)
    moving = [speed for speed in world.spins if abs(speed) > 0.05]
    assert moving[0] > 0 and min(moving) < 0
    assert not mission.line_lost


def test_ligne_introuvable_arrete_le_robot() -> None:
    mission, watch, world = _align(lambda world: None)
    assert mission.line_lost
    assert watch.states()[-1] == RobotState.ALIGNING_LINE.name
    assert (world.left, world.right) == (0.0, 0.0), "moteurs coupés"


def test_alignement_qui_n_y_arrive_pas_finit_par_repartir() -> None:
    """Ligne visible mais jamais centrée : le suivi de ligne prend le relais au lieu de rester bloqué."""
    mission, watch, _ = _align(lambda world: 150, max_steps=400)
    states = watch.states()
    assert not mission.line_lost
    last_aligning = len(states) - 1 - states[::-1].index(RobotState.ALIGNING_LINE.name)
    assert last_aligning < len(states) - 1, "il finit par sortir de l'alignement"


# Scénarios complets
def test_obstacle_au_milieu_d_une_arete_contourne_et_termine_la_livraison() -> None:
    world = World((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)])
    mission, recorder, grid = _run(world, (0, 0), (0, 2), Direction.SOUTH)

    assert grid.blocked_edges == [((0, 1), (0, 2))]
    assert mission.destination_reached and mission.returned_home and not mission.no_route
    assert world.at((0, 0)) < 4.0, "le robot est physiquement revenu au point de départ"
    assert world.closest >= 18.0, "il n'a jamais touché l'obstacle"

    nodes = recorder.nodes()
    assert nodes[:2] == [(0, 0), (0, 1)]
    assert (0, 2) in nodes
    assert frozenset(((0, 1), (0, 2))) not in _edges(nodes), "ni à l'aller ni au retour"
    states = [s.state for s in recorder.statuses]
    assert RobotState.DELIVERING.name in states, "la livraison a bien lieu au point B"
    delivering = next(s for s in recorder.statuses if s.state == RobotState.DELIVERING.name)
    assert delivering.position == (0, 2)


def test_arete_publiee_pour_l_interface_des_le_blocage() -> None:
    world = World((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)])
    _, recorder, _ = _run(world, (0, 0), (0, 2), Direction.SOUTH)
    assert recorder.statuses[0].blocked_edges == []
    assert recorder.statuses[-1].blocked_edges == [((0, 1), (0, 2))]
    first = next(s for s in recorder.statuses if s.blocked_edges)
    assert first.state == RobotState.TURNING_180.name, "publié dès le demi-tour de retour au nœud"
    assert first.route[-1] == (0, 2), "le nouveau chemin mène à B"


def test_loin_du_nœud_le_retour_se_fait_a_la_camera() -> None:
    """Assez loin pour que la caméra revoie le noeud, donc pas besoin de revenir à l'aveugle."""
    world = World((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)])
    mission, watch, _ = _watched(world, (0, 1))
    assert watch.state_after_u_turn() == RobotState.FOLLOWING.name
    assert watch.offset is not None and watch.offset < 3.0, "tourne au centre du nœud"
    assert mission.returned_home


def _close_to_node(**kwargs: Any) -> World:
    """Obstacle à 24 cm du noeud : le robot s'arrête à 4 cm du noeud, trop près pour que la caméra le revoie."""
    world = World((0, 0), Direction.SOUTH, obstacles=[(0, 1 + 24.0 / SPACING)], **kwargs)
    assert 24.0 - utils.OBSTACLE_STOP_CM < world.camera_reach
    return world


def test_tout_pres_du_nœud_le_retour_se_fait_a_l_aveugle_sur_la_distance_parcourue() -> None:
    world = _close_to_node()
    mission, watch, grid = _watched(world, (0, 1))
    assert grid.blocked_edges == [((0, 1), (0, 2))]
    assert watch.state_after_u_turn() == RobotState.APPROACHING_INTERSECTION.name
    assert watch.offset is not None and watch.offset < 3.0, "revient au centre, sans le dépasser"
    assert mission.destination_reached and mission.returned_home
    assert world.at((0, 0)) < 4.0
    assert frozenset(((0, 1), (0, 2))) not in _edges(watch.nodes())


def test_retour_aveugle_ne_compte_pas_le_nœud_une_seconde_fois_en_le_quittant() -> None:
    """Pire cas : la caméra voit encore le croisement sous le robot quand il repart."""
    world = _close_to_node(camera_nearest=-1.0)
    mission, watch, grid = _watched(world, (0, 1))
    assert grid.blocked_edges == [((0, 1), (0, 2))], "aucune arête coupée par erreur"
    assert mission.destination_reached and mission.returned_home
    assert world.at((0, 0)) < 4.0


def test_nœud_deja_visible_apres_le_demi_tour_detecte_quand_meme() -> None:
    """La caméra voit plus loin que prévu, le noeud est visible dès la fin du demi-tour."""
    class FarSighted(World):
        def step(self) -> bool:
            if self.get_distance_cm() <= utils.OBSTACLE_STOP_CM:
                self.camera_reach = 15.0
            return super().step()

    world = FarSighted((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)])
    mission, watch, _ = _watched(world, (0, 1))
    assert watch.offset is not None and watch.offset < 15.0, "ne passe pas le nœud sans le voir"
    assert mission.destination_reached


def test_obstacle_devant_le_robot_au_depart_tourne_sur_place_sans_demi_tour() -> None:
    world = World((0, 0), Direction.SOUTH, obstacles=[(0, 18.0 / SPACING)], max_steps=6000)
    mission, recorder, grid = _run(world, (0, 0), (0, 2), Direction.SOUTH)
    assert grid.blocked_edges == [((0, 0), (0, 1))]
    assert mission.destination_reached and mission.returned_home
    # Il repart du noeud avec une rotation de 90°, sans demi-tour inutile vu qu'il n'a pas bougé
    states = [s.state for s in recorder.statuses]
    assert RobotState.TURNING_MINUS_90.name in states or RobotState.TURNING_90.name in states
    delivery = states.index(RobotState.DELIVERING.name)
    assert RobotState.TURNING_180.name not in states[:delivery]


def test_obstacle_au_retour_recalcule_vers_le_point_de_depart() -> None:
    class ObstacleAfterDelivery(World):
        def step(self) -> bool:
            if self.at((0, 2)) < 2.0 and not self.obstacles and self.t > 5:
                self.obstacles.append((0, 0.5))
            return super().step()

    world = ObstacleAfterDelivery((0, 0), Direction.SOUTH)
    mission, recorder, grid = _run(world, (0, 0), (0, 2), Direction.SOUTH)
    assert grid.blocked_edges == [((0, 0), (0, 1))]
    assert mission.returned_home and mission.position == (0, 0)
    assert world.at((0, 0)) < 4.0
    return_nodes = recorder.nodes()[recorder.nodes().index((0, 2)):]
    assert frozenset(((0, 0), (0, 1))) not in _edges(return_nodes)


def test_retour_apres_detour_ne_repasse_pas_par_le_detour_inutile() -> None:
    """Le retour est recalculé, il ne refait pas le chemin de l'aller."""
    world = World((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)])
    _, recorder, _ = _run(world, (0, 0), (0, 2), Direction.SOUTH)
    nodes = recorder.nodes()
    back = nodes[nodes.index((0, 2)):]
    assert back[-1] == (0, 0) and len(back) == 5, "le chemin de retour le plus court sans l'arête bloquée"


def test_deux_obstacles_deux_aretes() -> None:
    world = World((0, 0), Direction.SOUTH, obstacles=[(0, 1.5), (1, 1.5)])
    mission, _, grid = _run(world, (0, 0), (0, 2), Direction.SOUTH)
    assert grid.blocked_edges == [((0, 1), (0, 2)), ((1, 1), (1, 2))]
    assert mission.destination_reached and mission.returned_home
    assert world.at((0, 0)) < 4.0
    assert world.closest >= 18.0


def test_aucun_chemin_arrete_la_mission_sur_place() -> None:
    grid = Grid()
    grid.block_edge((0, 2), (1, 2))
    grid.block_edge((0, 2), (0, 3))
    world = World((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)])
    mission, _, grid = _run(world, (0, 0), (0, 2), Direction.SOUTH, grid)
    assert mission.no_route and not mission.destination_reached
    assert ((0, 1), (0, 2)) in grid.blocked_edges
    assert world.left == world.right == 0.0
    assert world.closest >= 18.0


def test_obstacle_de_chaque_cote_arrete_la_mission() -> None:
    """Robot coincé : arête bloquée devant et un obstacle arrive derrière pendant le retour au noeud."""
    class Trap(World):
        def step(self) -> bool:
            if self.obstacles == [(0, 1.5)] and self.angle % 360 == 0 and self.t > 5:
                self.obstacles.append((0, 1.0))
            return super().step()

    world = Trap((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)], max_steps=1500)
    mission, _, _ = _run(world, (0, 0), (0, 2), Direction.SOUTH)
    assert mission.no_route
    assert world.left == world.right == 0.0


def test_obstacle_passager_ne_coupe_pas_l_arete() -> None:
    class Passing(World):
        def get_distance_cm(self) -> Optional[int]:
            return 15 if 30 <= self.steps < 30 + 8 else 200  # 0,5 s

    world = Passing((0, 0), Direction.SOUTH)
    mission, recorder, grid = _run(world, (0, 0), (0, 2), Direction.SOUTH)
    assert grid.blocked_edges == []
    assert mission.destination_reached and mission.returned_home
    assert any(s.obstacle_state == "BLOCKED" for s in recorder.statuses), "le robot s'est bien arrêté"
    assert recorder.nodes() == [(0, 0), (0, 1), (0, 2), (0, 1), (0, 0)]


def test_pas_de_mesure_pas_d_arete_coupee() -> None:
    class Mute(World):
        def get_distance_cm(self) -> Optional[int]:
            return None

    world = Mute((0, 0), Direction.SOUTH)
    mission, _, grid = _run(world, (0, 0), (0, 2), Direction.SOUTH)
    assert grid.blocked_edges == [] and mission.returned_home


def test_blocage_pendant_un_virage_ne_coupe_rien() -> None:
    """Un obstacle vu pendant une rotation n'est pas forcément sur la ligne suivie."""
    class SeenWhileTurning(World):
        def get_distance_cm(self) -> Optional[int]:
            return 15 if abs(self.left - self.right) > 1e-9 else 200

    world = SeenWhileTurning((0, 0), Direction.NORTH)
    mission, _, grid = _run(world, (0, 0), (0, 2), Direction.NORTH)
    assert grid.blocked_edges == [] and mission.returned_home


def test_sans_arete_bloquee_retour_par_le_meme_chemin() -> None:
    world = World((0, 0), Direction.EAST)
    _, recorder, grid = _run(world, (0, 0), (2, 1), Direction.EAST)
    nodes = recorder.nodes()
    middle = nodes.index((2, 1))
    assert nodes[middle:] == nodes[:middle + 1][::-1]
    assert grid.blocked_edges == []


# Programme complet et interface

class Paced(World):
    finished = False

    def step(self) -> bool:
        time.sleep(0.0005)
        super().step()
        return not self.finished


@pytest.fixture
def served() -> Any:
    world = Paced((0, 0), Direction.SOUTH, obstacles=[(0, 1.5)], max_steps=10 ** 9)
    server = TelemetryServer(port=0)
    thread = threading.Thread(target=serve_missions, args=(world, world, server, world))
    thread.start()
    yield world, server._server.server_address[1]
    world.finished = True
    thread.join(10)
    server.close()
    assert not thread.is_alive()


def _post(port: int, path: str, payload: object) -> int:
    request = urllib.request.Request(f"http://127.0.0.1:{port}{path}", json.dumps(payload).encode(),
                                     {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request) as response:
            return response.status
    except urllib.error.HTTPError as error:
        return error.code


def _state(port: int) -> dict:
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/state.json") as response:
        return json.loads(response.read())


def _wait(condition: Any, timeout: float = 30.0) -> None:
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, "condition jamais atteinte"
        time.sleep(0.005)


MISSION = {"start": [0, 0], "destination": [0, 2], "heading": "SOUTH"}


def test_arete_conservee_entre_missions_puis_debloquee(served: tuple) -> None:
    world, port = served
    _wait(lambda: world.steps > 2)
    assert _state(port)["blocked_edges"] == []

    assert _post(port, "/mission", MISSION) == 200
    _wait(lambda: _state(port)["blocked_edges"])
    assert _post(port, "/map/reset", {}) == 409, "pas pendant une mission"
    _wait(lambda: _state(port)["state"] == "IDLE" and _state(port)["returned_home"])
    expected = [[[0, 1], [0, 2]]]
    assert _state(port)["blocked_edges"] == expected, "reste affichée à la fin de la mission"

    # Mission suivante, avec la vraie orientation du robot : il doit éviter l'arête dès le départ
    assert _post(port, "/mission", {**MISSION, "heading": _state(port)["heading"]}) == 200
    _wait(lambda: _state(port)["state"] != "IDLE")
    route = [tuple(n) for n in _state(port)["route"]]
    assert frozenset(((0, 1), (0, 2))) not in _edges(route)
    _wait(lambda: _state(port)["state"] == "IDLE" and _state(port)["returned_home"])

    assert _post(port, "/map/reset", {}) == 200
    _wait(lambda: _state(port)["blocked_edges"] == [])
    state = _state(port)
    assert state["route"] == [] and state["start"] is None and not state["returned_home"], "carte vierge"


def test_point_de_depart_isole_signale_sans_planter(served: tuple) -> None:
    """Deux obstacles autour du départ : plus de chemin, ni pendant la mission ni à la suivante."""
    world, port = served
    _wait(lambda: world.steps > 2)
    world.obstacles[:] = [(0, 18.0 / SPACING), (18.0 / SPACING, 0)]
    order = {"start": [0, 0], "destination": [0, 1], "heading": "SOUTH"}

    assert _post(port, "/mission", order) == 200
    _wait(lambda: _state(port)["no_route"] and _state(port)["state"] == "IDLE")
    state = _state(port)
    assert state["blocked_edges"] == [[[0, 0], [0, 1]], [[0, 0], [1, 0]]]
    assert world.left == world.right == 0.0

    # Ordre impossible dès le lancement, le programme ne doit pas planter pour autant
    world.obstacles.clear()
    assert _post(port, "/mission", {**order, "destination": [4, 4], "heading": state["heading"]}) == 200
    steps = world.steps
    _wait(lambda: world.steps > steps + 30)
    assert _state(port)["no_route"] and _state(port)["state"] == "IDLE"
    assert world.left == world.right == 0.0, "moteurs coupés"

    # Une fois les arêtes débloquées, la mission suivante doit marcher
    assert _post(port, "/map/reset", {}) == 200
    _wait(lambda: _state(port)["blocked_edges"] == [])
    assert _post(port, "/mission", {**order, "heading": state["heading"]}) == 200
    _wait(lambda: _state(port)["returned_home"] and _state(port)["state"] == "IDLE")
    assert not _state(port)["no_route"]
