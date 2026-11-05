"""Tests de la couche robot, sans moteurs ni caméra."""
from __future__ import annotations

import json
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from unittest import mock

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

sys.modules.setdefault("serial", mock.MagicMock())  # pyserial absent hors du robot

import EiNav  # noqa: E402
from connections.base import RobotConnection  # noqa: E402
from planning.direction import Direction  # noqa: E402
from planning.navigation import Grid, Mission, Navigation  # noqa: E402
from robot import utils as robot_utils  # noqa: E402
from robot.control import Control  # noqa: E402
from robot.state_machine import get_turn_state, run_figure_eight, run_state_machine, serve_missions  # noqa: E402
from robot.states import Mode, RobotState  # noqa: E402
from robot.telemetry import (  # noqa: E402
    FigureEightRequest,
    MissionRequest,
    TelemetryServer,
    make_status,
    parse_mission,
)
from vision.image_processor import ImageProcessor, LineDetection  # noqa: E402


class FakeSerial:
    def __init__(self) -> None:
        self.commands: List[Tuple[float, float, float, float]] = []

    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        self.commands.append((fl, fr, rr, rl))

    def get_distance_cm(self) -> Optional[int]:
        return 200  # rien devant le robot

    def disconnect(self) -> None:
        pass


# --- Mission ---------------------------------------------------------------------------------

def test_mission_leve_le_flag_puis_fait_demi_tour_et_revient() -> None:
    mission = Mission([(0, 0), (1, 0), (1, 1)])

    assert mission.on_intersection() == Direction.EAST
    assert mission.on_intersection() == Direction.SOUTH
    assert not mission.destination_reached

    # Arrivée : flag levé, puis direction opposée à celle d'arrivée.
    assert mission.on_intersection() == Direction.NORTH
    assert mission.destination_reached
    assert not mission.returned_home

    assert mission.on_intersection() == Direction.WEST
    assert mission.on_intersection() is None
    assert mission.returned_home
    assert mission.position == (0, 0)


def test_mission_depart_egal_arrivee() -> None:
    mission = Mission([(2, 2)])
    assert mission.on_intersection() is None
    assert mission.destination_reached and mission.returned_home


def test_retour_commence_par_un_demi_tour() -> None:
    assert get_turn_state(Direction.SOUTH, Direction.NORTH) == RobotState.TURNING_180


# --- Rampes exponentielles -------------------------------------------------------------------

def _make_control() -> Tuple[Control, FakeSerial]:
    serial = FakeSerial()
    return Control(serial), serial  # type: ignore[arg-type]


def test_acceleration_et_deceleration_exponentielles() -> None:
    control, serial = _make_control()
    dt = 0.05
    for i in range(40):
        control.aim(i * dt, 0.0)
    speeds = [c[0] for c in serial.commands]
    assert speeds[1] < speeds[2] < robot_utils.BASE_SPEED
    assert speeds[-1] == robot_utils.BASE_SPEED
    # Écart à la consigne divisé par un facteur constant : c'est une exponentielle.
    ratios = [(robot_utils.BASE_SPEED - speeds[i + 1]) / (robot_utils.BASE_SPEED - speeds[i]) for i in range(1, 6)]
    assert ratios == pytest.approx([np.exp(-dt / robot_utils.RAMP_TIME_CONSTANT)] * 5)

    t = 40 * dt
    stopped = False
    for i in range(40):
        stopped = control.stop(t + i * dt)
        if stopped:
            break
    assert stopped
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)


@pytest.mark.parametrize("half_turn", [False, True])
def test_rotation_conserve_angle_malgre_les_rampes(half_turn: bool) -> None:
    control, serial = _make_control()
    dt = 0.01
    t = 0.0
    while not control.turn(t, False, 0.0, half_turn=half_turn):
        t += dt
    integral = sum(c[0] for c in serial.commands) * dt
    duration = robot_utils.TURN_180_DURATION if half_turn else robot_utils.TURN_90_DURATION
    assert integral == pytest.approx(robot_utils.TURN_SPEED * duration, rel=0.02)
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)


def test_rotation_a_gauche_inverse_les_roues() -> None:
    control, serial = _make_control()
    control.turn(0.0, True, 0.0)
    control.turn(0.1, True, 0.0)
    fl, fr, rr, rl = serial.commands[-1]
    assert fl < 0 < fr and rl == fl and rr == fr


def test_approche_compense_la_distance_de_rampe() -> None:
    control, serial = _make_control()
    dt = 0.01
    t = 0.0
    while t < 2.0:
        control.aim(t, 0.0)
        t += dt
    serial.commands.clear()
    duration = control.approach_duration(0.9)
    assert 0 < duration < 0.9
    end = t + duration
    while t < end:
        control.forward(t)
        t += dt
    while not control.stop(t):
        t += dt
    distance = sum(c[0] for c in serial.commands) * dt
    assert distance == pytest.approx(robot_utils.FORWARD_SPEED * 0.9, rel=0.1)


def test_correction_de_direction_non_filtree() -> None:
    control, serial = _make_control()
    control.aim(0.0, 1.0)
    fl, fr, _, _ = serial.commands[-1]
    assert fl - fr == pytest.approx(2 * robot_utils.KP)


# --- Machine à états -------------------------------------------------------------------------

class FakeConnection(RobotConnection):
    """Horloge simulée. Rend une image par pas ; `max_steps` borne les tests."""

    def __init__(self, dt: float = 0.05, max_steps: int = 20000, drop_first: int = 0) -> None:
        self.t = 0.0
        self.dt = dt
        self.steps = 0
        self.max_steps = max_steps
        self.drop_first = drop_first
        self.published = 0

    def get_time(self) -> float:
        return self.t

    def step(self) -> bool:
        self.t += self.dt
        self.steps += 1
        return self.steps < self.max_steps

    def get_frame(self) -> Optional[np.ndarray]:
        if self.steps <= self.drop_first:
            return None
        return np.zeros((128, 160, 3), np.uint8)

    def publish_frame(self, frame: np.ndarray) -> None:
        self.published += 1


class FakeProcessor(ImageProcessor):
    """Intersection vue périodiquement, 3 images de suite."""

    def __init__(self, period: int = 40) -> None:
        self.period = period
        self.calls = 0

    def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
        self.calls += 1
        seen = self.calls % self.period < 3
        return LineDetection(80, seen, 160, 76, np.zeros((52, 160), np.uint8))


class Recorder:
    stop_requested = False

    def __init__(self) -> None:
        self.statuses: list = []

    def publish(self, status: object) -> None:
        self.statuses.append(status)


def test_aller_retour_complet() -> None:
    serial = FakeSerial()
    connection = FakeConnection()
    recorder = Recorder()
    mission = run_state_machine(connection, serial, (0, 1), (2, 2), Direction.SOUTH,  # type: ignore[arg-type]
                                processor=FakeProcessor(), telemetry=recorder)  # type: ignore[arg-type]

    assert connection.steps < connection.max_steps, "le robot n'a jamais terminé"
    assert mission.destination_reached and mission.returned_home
    assert mission.position == (0, 1)
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)
    assert connection.published > 0

    states = [s.state for s in recorder.statuses]
    assert RobotState.TURNING_180.name in states

    # La livraison a lieu sur la destination, juste après la levée du flag, puis le demi-tour.
    delivery = states.index(RobotState.DELIVERING.name)
    assert recorder.statuses[delivery].position == (2, 2)
    assert recorder.statuses[delivery].destination_reached
    assert recorder.statuses[delivery - 1].destination_reached is False
    first_u_turn = states.index(RobotState.TURNING_180.name)
    assert first_u_turn > delivery and recorder.statuses[first_u_turn].position == (2, 2)
    assert recorder.statuses[-1].state == RobotState.STOPPED.name
    assert recorder.statuses[-1].returned_home


class ScriptedProcessor(FakeProcessor):
    """Intersection vue uniquement sur les images dont le numéro est donné (à partir de 1)."""

    def __init__(self, seen_on: Tuple[int, ...] = ()) -> None:
        super().__init__()
        self.seen_on = seen_on

    def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
        self.calls += 1
        return LineDetection(80, self.calls in self.seen_on, 160, 76, np.zeros((52, 160), np.uint8))


def _run_without_seeing_intersection(start: tuple, end: tuple, heading: Direction) -> Tuple[Recorder, FakeSerial]:
    recorder, serial = Recorder(), FakeSerial()
    run_state_machine(FakeConnection(max_steps=100), serial, start, end, heading,  # type: ignore[arg-type]
                      processor=ScriptedProcessor(), telemetry=recorder)  # type: ignore[arg-type]
    return recorder, serial


def test_depart_sans_voir_le_premier_croisement() -> None:
    """Posé sur le croisement, le robot ne peut pas le voir : il part sans l'attendre."""
    recorder, serial = _run_without_seeing_intersection((0, 0), (0, 2), Direction.SOUTH)
    assert recorder.statuses[0].position == (0, 0), "sur le point de départ dès le lancement"
    states = [s.state for s in recorder.statuses]
    assert RobotState.APPROACHING_INTERSECTION.name not in states
    assert states[-1] == RobotState.FOLLOWING.name
    assert recorder.statuses[-1].next_node == (0, 1), "le premier nœud est déjà compté"
    assert any(c[0] > 0 and c[0] == c[1] for c in serial.commands), "le robot avance"


def test_virage_sur_place_au_depart_si_mal_oriente() -> None:
    recorder, _ = _run_without_seeing_intersection((0, 0), (0, 2), Direction.NORTH)
    states = [s.state for s in recorder.statuses]
    assert RobotState.TURNING_180.name in states
    assert RobotState.APPROACHING_INTERSECTION.name not in states
    assert recorder.statuses[-1].heading == Direction.SOUTH.name


def test_chaque_croisement_compte_une_seule_fois() -> None:
    """(0, 0) -> (0, 2) et retour : 4 croisements à voir, le départ n'étant vu qu'au retour."""
    recorder = Recorder()
    connection = FakeConnection()
    mission = run_state_machine(connection, FakeSerial(), (0, 0), (0, 2), Direction.SOUTH,  # type: ignore[arg-type]
                                processor=FakeProcessor(), telemetry=recorder)  # type: ignore[arg-type]
    assert mission.returned_home and mission.position == (0, 0)
    states = [s.state for s in recorder.statuses]
    approaches = sum(1 for before, after in zip(states, states[1:])
                     if after == RobotState.APPROACHING_INTERSECTION.name and before != after)
    assert approaches == 4


def test_croisement_vu_au_depart_ignore_jusqu_a_l_avoir_quitte() -> None:
    """Si la caméra voit quand même le croisement de départ, il ne compte pas comme le suivant."""
    recorder = Recorder()
    run_state_machine(FakeConnection(max_steps=8), FakeSerial(), (0, 0), (0, 2), Direction.SOUTH,  # type: ignore[arg-type]
                      processor=ScriptedProcessor(seen_on=(1, 2, 3, 4)), telemetry=recorder)  # type: ignore[arg-type]
    assert RobotState.APPROACHING_INTERSECTION.name not in [s.state for s in recorder.statuses]


def test_image_perdue_ne_fait_pas_planter() -> None:
    connection = FakeConnection(drop_first=5)
    mission = run_state_machine(connection, FakeSerial(), (0, 0), (0, 0), Direction.SOUTH,  # type: ignore[arg-type]
                                processor=FakeProcessor())
    assert mission.returned_home


def test_chemin_impossible() -> None:
    with pytest.raises(ValueError):
        run_state_machine(FakeConnection(), FakeSerial(), (0, 0), (9, 9), Direction.SOUTH,  # type: ignore[arg-type]
                          processor=FakeProcessor())


@pytest.mark.parametrize("heading,state,used,other", [
    (Direction.NORTH, RobotState.TURNING_180, "TURN_180_DURATION", "TURN_90_DURATION"),
    (Direction.EAST, RobotState.TURNING_MINUS_90, "TURN_90_DURATION", "TURN_180_DURATION"),
    (Direction.WEST, RobotState.TURNING_90, "TURN_90_DURATION", "TURN_180_DURATION"),
])
def test_chaque_virage_utilise_sa_duree(heading: Direction, state: RobotState, used: str, other: str,
                                        monkeypatch: pytest.MonkeyPatch) -> None:
    """Départ de (0, 0) vers le sud : l'orientation initiale impose le virage testé."""
    def steps_in_state() -> int:
        recorder, _ = _run_without_seeing_intersection((0, 0), (0, 2), heading)
        return sum(1 for s in recorder.statuses if s.state == state.name)

    monkeypatch.setattr(robot_utils, used, 1.0)
    monkeypatch.setattr(robot_utils, other, 0.5)
    reference = steps_in_state()
    assert reference > 0

    monkeypatch.setattr(robot_utils, other, 2.0)
    assert steps_in_state() == reference, "l'autre durée n'a aucun effet"

    monkeypatch.setattr(robot_utils, used, 2.0)
    assert steps_in_state() > reference, "cette durée allonge le virage"


# --- Livraison : tour complet à destination ----------------------------------------------------

class SpinSerial(FakeSerial):
    """Rotation envoyée aux roues (> 0 : à droite), rangée par état du robot."""

    def __init__(self, recorder: Recorder, dt: float) -> None:
        super().__init__()
        self.recorder = recorder
        self.dt = dt
        self.rotation: dict = {}

    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        super().send_wheel_speeds(fl, fr, rr, rl)
        if self.recorder.statuses and fl == -fr:
            state = self.recorder.statuses[-1].state
            self.rotation[state] = self.rotation.get(state, 0.0) + fl * self.dt


def _delivery(start: tuple = (0, 0), end: tuple = (0, 2)) -> Tuple[Mission, Recorder, SpinSerial]:
    recorder = Recorder()
    connection = FakeConnection(dt=0.02)
    serial = SpinSerial(recorder, connection.dt)
    mission = run_state_machine(connection, serial, start, end, Direction.SOUTH,  # type: ignore[arg-type]
                                processor=FakeProcessor(period=120), telemetry=recorder)  # type: ignore[arg-type]
    return mission, recorder, serial


def test_tour_complet_a_destination_avant_le_retour() -> None:
    mission, recorder, serial = _delivery()
    assert mission.returned_home and mission.position == (0, 0)
    half_turn = robot_utils.TURN_SPEED * robot_utils.TURN_180_DURATION
    # La fin d'un demi-tour se rattache à l'état suivant : on vérifie le total, puis la répartition.
    assert serial.rotation[RobotState.DELIVERING.name] == pytest.approx(2 * half_turn, rel=0.05)
    total = serial.rotation[RobotState.DELIVERING.name] + serial.rotation[RobotState.TURNING_180.name]
    assert total == pytest.approx(3 * half_turn, rel=0.02), "360° de livraison + demi-tour du retour"


def test_livraison_affichee_uniquement_pendant_le_tour() -> None:
    _, recorder, _ = _delivery((0, 0), (0, 3))
    delivering = [s for s in recorder.statuses if s.state == RobotState.DELIVERING.name]
    assert delivering
    assert {s.position for s in delivering} == {(0, 3)}, "seulement à destination, pas aux autres nœuds"
    assert all(s.destination_reached and not s.returned_home for s in delivering)

    states = [s.state for s in recorder.statuses]
    entries = sum(1 for before, after in zip(states, states[1:])
                  if after == RobotState.DELIVERING.name and before != after)
    assert entries == 1, "une seule livraison par mission, pas de tour au retour sur A"
    assert states[states.index(RobotState.DELIVERING.name) + len(delivering)] == RobotState.TURNING_180.name


def test_duree_de_la_livraison_suit_le_reglage_du_demi_tour(monkeypatch: pytest.MonkeyPatch) -> None:
    def spin_time() -> float:
        _, recorder, _ = _delivery()
        return sum(1 for s in recorder.statuses if s.state == RobotState.DELIVERING.name) * 0.02

    reference = spin_time()
    monkeypatch.setattr(robot_utils, "TURN_180_DURATION", robot_utils.TURN_180_DURATION + 1.0)
    assert spin_time() == pytest.approx(reference + 2.0, abs=0.2)


def test_arret_possible_pendant_la_livraison() -> None:
    class StopDuringDelivery(Recorder):
        def publish(self, status: object) -> None:
            super().publish(status)
            self.stop_requested = status.state == RobotState.DELIVERING.name  # type: ignore[attr-defined]

    recorder, serial = StopDuringDelivery(), FakeSerial()
    mission = run_state_machine(FakeConnection(), serial, (0, 0), (0, 1), Direction.SOUTH,  # type: ignore[arg-type]
                                processor=FakeProcessor(), telemetry=recorder)  # type: ignore[arg-type]
    assert mission.destination_reached and not mission.returned_home
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)


def test_page_affiche_livraison_en_cours() -> None:
    assert "Livraison en cours" in (ROOT / "frontend" / "live.html").read_text(encoding="utf-8")


# --- Parcours en 8 ---------------------------------------------------------------------------

def _run_figure_eight(seen_on: Tuple[int, ...] = (), steps: int = 200) -> Tuple[Recorder, FakeSerial]:
    recorder, serial = Recorder(), FakeSerial()
    run_figure_eight(FakeConnection(max_steps=steps), serial, ScriptedProcessor(seen_on),  # type: ignore[arg-type]
                     recorder)  # type: ignore[arg-type]
    return recorder, serial


def test_huit_traverse_les_croisements_sans_s_arreter() -> None:
    recorder, serial = _run_figure_eight(seen_on=(50, 51, 52, 120, 121, 122))
    states = [s.state for s in recorder.statuses]
    approaches = sum(1 for before, after in zip(states, states[1:])
                     if after == RobotState.APPROACHING_INTERSECTION.name and before != after)
    assert approaches == 2
    assert set(states) == {RobotState.FOLLOWING.name, RobotState.APPROACHING_INTERSECTION.name}
    assert states[-1] == RobotState.FOLLOWING.name, "reprend le suivi après le croisement"
    assert all(c[0] > 0 for c in serial.commands[1:-1]), "ne s'arrête jamais à un croisement"
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0), "moteurs coupés en fin de boucle"


def test_huit_annonce_son_mode_sans_grille() -> None:
    recorder, _ = _run_figure_eight()
    status = recorder.statuses[0]
    assert status.mode == Mode.HUIT.name
    assert status.start is None and status.position is None and status.route == []


def test_huit_ignore_le_croisement_sur_lequel_il_est_pose() -> None:
    recorder, _ = _run_figure_eight(seen_on=(1, 2, 3, 4), steps=8)
    assert RobotState.APPROACHING_INTERSECTION.name not in [s.state for s in recorder.statuses]


def test_huit_ne_roule_pas_sans_image() -> None:
    serial = FakeSerial()
    run_figure_eight(FakeConnection(max_steps=5, drop_first=10), serial, ScriptedProcessor(),  # type: ignore[arg-type]
                     Recorder())  # type: ignore[arg-type]
    assert all(c == (0.0, 0.0, 0.0, 0.0) for c in serial.commands)


# --- Ligne perdue ----------------------------------------------------------------------------

class LineScriptProcessor(FakeProcessor):
    """Ligne vue seulement quand `visible(numéro d'image)` est vrai, à partir de l'image 1."""

    def __init__(self, visible: Any) -> None:
        super().__init__()
        self.visible = visible

    def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
        self.calls += 1
        cx = 80 if self.visible(self.calls) else None
        return LineDetection(cx, False, 160, 76, np.zeros((52, 160), np.uint8))


def _brief_loss(dt: float = 0.05) -> int:
    """Nombre d'images sans ligne qui reste sous le délai de ligne perdue."""
    return int(0.9 * robot_utils.LINE_LOST_TIMEOUT / dt)


def _grid_mission(visible: Any, heading: Direction = Direction.SOUTH, steps: int = 400) -> Tuple[Mission, FakeConnection,
                                                                                                FakeSerial]:
    connection, serial = FakeConnection(max_steps=steps), FakeSerial()
    mission = run_state_machine(connection, serial, (0, 0), (0, 2), heading,  # type: ignore[arg-type]
                                processor=LineScriptProcessor(visible))
    return mission, connection, serial


def test_ligne_perdue_plus_d_une_seconde_arrete_la_mission() -> None:
    last_seen = 20
    mission, connection, serial = _grid_mission(lambda i: i <= last_seen)
    assert mission.line_lost and not mission.returned_home
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)
    # Première image sans ligne : 21. L'arrêt arrive à 1 s, à une image près (l'horloge simulée
    # additionne des flottants : l'image de la 20e période peut déjà dépasser 1 s d'un arrondi).
    lost_for = (connection.steps - (last_seen + 1)) * connection.dt
    assert robot_utils.LINE_LOST_TIMEOUT <= lost_for <= robot_utils.LINE_LOST_TIMEOUT + 2 * connection.dt


def test_ligne_perdue_moins_d_une_seconde_ne_change_rien() -> None:
    """Sans ligne un peu moins longtemps que le délai, puis la ligne revient."""
    mission, connection, _ = _grid_mission(lambda i: not 20 < i <= 20 + _brief_loss(), steps=200)
    assert not mission.line_lost
    assert connection.steps == 200, "le robot roule toujours"


def test_pertes_breves_repetees_ne_s_additionnent_pas() -> None:
    mission, _, _ = _grid_mission(lambda i: i % _brief_loss() == 0, steps=300)
    assert not mission.line_lost, "une image avec la ligne de temps en temps suffit à continuer"


def test_ligne_invisible_pendant_un_virage_n_arrete_pas() -> None:
    """Demi-tour de 3 s au départ, sans voir la ligne : seul le suivi compte."""
    mission, connection, _ = _grid_mission(lambda i: i > 70, heading=Direction.NORTH, steps=200)
    assert not mission.line_lost
    assert connection.steps == 200


def test_compteur_demarre_a_l_entree_en_suivi() -> None:
    recorder = Recorder()
    connection = FakeConnection(max_steps=400)
    mission = run_state_machine(connection, FakeSerial(), (0, 0), (0, 2), Direction.NORTH,  # type: ignore[arg-type]
                                processor=LineScriptProcessor(lambda i: False), telemetry=recorder)  # type: ignore[arg-type]
    assert mission.line_lost
    searching = sum(1 for s in recorder.statuses if s.state == RobotState.SEARCHING_LINE.name)
    assert robot_utils.LINE_LOST_TIMEOUT < searching * connection.dt <= robot_utils.LINE_LOST_TIMEOUT + 3 * connection.dt
    assert RobotState.SEARCHING_LINE.name in [s.state for s in recorder.statuses]


def test_huit_s_arrete_si_la_ligne_est_perdue() -> None:
    connection, serial = FakeConnection(max_steps=400), FakeSerial()
    line_lost = run_figure_eight(connection, serial, LineScriptProcessor(lambda i: i <= 20),  # type: ignore[arg-type]
                                 Recorder())  # type: ignore[arg-type]
    assert line_lost is True
    assert connection.steps < 60
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)


def test_huit_continue_si_la_ligne_revient() -> None:
    connection = FakeConnection(max_steps=100)
    line_lost = run_figure_eight(connection, FakeSerial(), LineScriptProcessor(lambda i: not 20 < i <= 20 + _brief_loss()),  # type: ignore[arg-type]
                                 Recorder())  # type: ignore[arg-type]
    assert line_lost is False


class SideScriptProcessor(FakeProcessor):
    """Ligne vue à une abscisse spécifique (cx) pendant certains pas, puis None."""
    def __init__(self, cx_by_call: Dict[int, Optional[int]], default_cx: Optional[int] = None) -> None:
        super().__init__()
        self.cx_by_call = cx_by_call
        self.default_cx = default_cx

    def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
        self.calls += 1
        cx = self.cx_by_call.get(self.calls, self.default_cx)
        return LineDetection(cx, False, 160, 76, np.zeros((52, 160), np.uint8))


def test_huit_memorise_ligne_a_gauche_et_tourne_a_gauche() -> None:
    """Si la ligne disparaît à gauche (cx < 80), le robot passe en SEARCHING_LINE et pivote à gauche."""
    cx_map = {i: 30 for i in range(1, 11)}
    processor = SideScriptProcessor(cx_map, default_cx=None)
    connection, serial = FakeConnection(max_steps=15), FakeSerial()
    recorder = Recorder()
    run_figure_eight(connection, serial, processor, recorder)  # type: ignore[arg-type]

    states = [s.state for s in recorder.statuses]
    assert RobotState.SEARCHING_LINE.name in states
    assert any(cmd[0] < cmd[1] for cmd in serial.commands)


def test_huit_memorise_ligne_a_droite_et_tourne_a_droite() -> None:
    """Si la ligne disparaît à droite (cx > 80), le robot passe en SEARCHING_LINE et pivote à droite."""
    cx_map = {i: 130 for i in range(1, 11)}
    processor = SideScriptProcessor(cx_map, default_cx=None)
    connection, serial = FakeConnection(max_steps=15), FakeSerial()
    recorder = Recorder()
    run_figure_eight(connection, serial, processor, recorder)  # type: ignore[arg-type]

    states = [s.state for s in recorder.statuses]
    assert RobotState.SEARCHING_LINE.name in states
    assert any(cmd[0] > cmd[1] for cmd in serial.commands)


def test_huit_retrouve_la_ligne_et_reprend_following() -> None:
    """En mode huit, la ligne disparaît (SEARCHING_LINE) puis réapparaît (retour en FOLLOWING)."""
    cx_map = {i: 50 for i in range(1, 11)}
    cx_map.update({i: None for i in range(11, 15)})
    cx_map.update({i: 80 for i in range(15, 30)})
    processor = SideScriptProcessor(cx_map, default_cx=80)
    connection = FakeConnection(max_steps=25)
    recorder = Recorder()
    line_lost = run_figure_eight(connection, FakeSerial(), processor, recorder)  # type: ignore[arg-type]
    assert line_lost is False
    states = [s.state for s in recorder.statuses]
    assert RobotState.SEARCHING_LINE.name in states
    assert states[-1] == RobotState.FOLLOWING.name


def test_grille_apres_virage_cherche_ligne_et_reprend_following() -> None:
    """Après un virage, si la ligne n'est pas immédiatement en vue, le robot passe en SEARCHING_LINE."""
    call_count = 0
    class AfterTurnProcessor(FakeProcessor):
        def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
            nonlocal call_count
            call_count += 1
            cx = None if 44 <= call_count <= 47 else 80
            return LineDetection(cx, False, 160, 76, np.zeros((52, 160), np.uint8))

    connection = FakeConnection(max_steps=50)
    recorder = Recorder()
    mission = run_state_machine(connection, FakeSerial(), (0, 0), (0, 2), Direction.WEST,  # type: ignore[arg-type]
                                processor=AfterTurnProcessor(), telemetry=recorder)  # type: ignore[arg-type]
    states = [s.state for s in recorder.statuses]
    assert RobotState.SEARCHING_LINE.name in states
    assert states[-1] == RobotState.FOLLOWING.name
    assert not mission.line_lost


def test_grille_apres_virage_memorise_sens_et_arrete_apres_timeout() -> None:
    """Après un virage à gauche, si la ligne n'est jamais trouvée, recherche vers la gauche puis arrêt."""
    processor = LineScriptProcessor(lambda i: i <= 2)
    recorder = Recorder()
    serial = FakeSerial()
    mission = run_state_machine(FakeConnection(max_steps=200), serial, (0, 0), (0, 2), Direction.WEST,  # type: ignore[arg-type]
                                processor=processor, telemetry=recorder)  # type: ignore[arg-type]
    assert mission.line_lost
    states = [s.state for s in recorder.statuses]
    assert RobotState.SEARCHING_LINE.name in states
    assert any(cmd[0] < cmd[1] for cmd in serial.commands)


# --- Vision ----------------------------------------------------------------------------------

def _frame_with_cross() -> np.ndarray:
    frame = np.zeros((128, 160, 3), np.uint8)
    frame[:, 70:90] = 255
    frame[95:110, :] = 255
    return frame


def test_detection_et_annotation() -> None:
    processor = ImageProcessor()
    frame = _frame_with_cross()
    detection = processor.process_frame(frame)
    assert detection.is_intersection
    assert detection.cx is not None and 70 <= detection.cx <= 90

    original = frame.copy()
    annotated = processor.annotate(frame, detection)
    assert annotated.shape == frame.shape
    assert np.array_equal(frame, original), "l'image d'entrée ne doit pas être modifiée"
    assert not np.array_equal(annotated, frame)


# --- Tableau de bord -------------------------------------------------------------------------

def test_serveur_de_telemetrie() -> None:
    server = TelemetryServer(port=0)
    port = server._server.server_address[1]
    base = f"http://127.0.0.1:{port}"
    try:
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(base + "/state.json")
        assert error.value.code == 503

        mission = Mission(Navigation().generate_path(Grid(4), (0, 1), (1, 1), Direction.SOUTH), Direction.EAST)
        mission.on_intersection()
        server.publish(make_status(Mode.GRID, RobotState.FOLLOWING, 5, mission, True, False))
        with urllib.request.urlopen(base + "/state.json") as response:
            state = json.loads(response.read())
        assert state["state"] == "FOLLOWING"
        assert state["position"] == [0, 1]
        assert state["next_node"] == [1, 1]
        assert state["grid_nodes"] == 5

        with urllib.request.urlopen(base + "/") as response:
            page = response.read().decode("utf-8")
        assert "Livraison sur la grille" in page and "Suivi de ligne en 8" in page
    finally:
        server.close()


# --- Commande depuis l'interface -------------------------------------------------------------

def post(port: int, path: str, payload: object, content_type: str = "application/json") -> Tuple[int, dict]:
    request = urllib.request.Request(f"http://127.0.0.1:{port}{path}", json.dumps(payload).encode(),
                                     {"Content-Type": content_type})
    try:
        with urllib.request.urlopen(request) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def state_of(port: int) -> dict:
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/state.json") as response:
        return json.loads(response.read())


def wait_until(condition: object, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    while not condition():  # type: ignore[operator]
        assert time.monotonic() < deadline, "condition jamais atteinte"
        time.sleep(0.005)


MISSION = {"start": [0, 1], "destination": [1, 1], "heading": "SOUTH"}


def test_parse_mission_valide() -> None:
    expected = MissionRequest((0, 1), (1, 1), Direction.SOUTH)
    assert parse_mission(json.dumps(MISSION).encode(), 5) == expected
    assert parse_mission(json.dumps({**MISSION, "mode": "GRID"}).encode(), 5) == expected


def test_parse_mission_huit_sans_point_de_depart() -> None:
    assert parse_mission(b'{"mode": "HUIT"}', 5) == FigureEightRequest()


@pytest.mark.parametrize("body", [
    b"pas du json",
    b"[]",
    b"{}",
    json.dumps({**MISSION, "start": [5, 0]}).encode(),
    json.dumps({**MISSION, "start": [-1, 0]}).encode(),
    json.dumps({**MISSION, "start": [0.5, 0]}).encode(),
    json.dumps({**MISSION, "start": [True, 0]}).encode(),
    json.dumps({**MISSION, "start": "0,1"}).encode(),
    json.dumps({**MISSION, "destination": [0, 1]}).encode(),
    json.dumps({**MISSION, "heading": "HAUT"}).encode(),
    json.dumps({**MISSION, "heading": ["SOUTH"]}).encode(),
    json.dumps({**MISSION, "mode": "TOUPIE"}).encode(),
    json.dumps({**MISSION, "mode": ["HUIT"]}).encode(),
    json.dumps({"mode": "GRID"}).encode(),
])
def test_parse_mission_refuse_les_ordres_invalides(body: bytes) -> None:
    with pytest.raises(ValueError):
        parse_mission(body, 5)


class PacedConnection(FakeConnection):
    """Horloge simulée rapide, arrêtée sur demande du test."""

    def __init__(self) -> None:
        super().__init__(max_steps=10 ** 9)
        self.finished = False

    def step(self) -> bool:
        time.sleep(0.0005)
        super().step()
        return not self.finished


@pytest.fixture
def robot() -> object:
    server = TelemetryServer(port=0)
    serial, connection = FakeSerial(), PacedConnection()
    thread = threading.Thread(target=serve_missions, args=(connection, serial, server, FakeProcessor()))
    thread.start()
    port = server._server.server_address[1]
    wait_until(lambda: connection.steps > 2)
    yield port, serial, connection, server
    connection.finished = True
    thread.join(10)
    assert not thread.is_alive()
    server.close()


def test_le_robot_ne_bouge_pas_avant_demarrer(robot: tuple) -> None:
    port, serial, connection, _ = robot
    steps = connection.steps
    wait_until(lambda: connection.steps > steps + 100)
    assert serial.commands == [], "aucune commande moteur avant le clic sur Démarrer"
    assert connection.published > 0, "la caméra reste visible pour placer le robot"
    state = state_of(port)
    assert state["state"] == "IDLE"
    assert state["start"] is None and state["route"] == []
    assert state["grid_nodes"] == 5


def test_demarrer_execute_l_aller_retour_puis_attend(robot: tuple) -> None:
    port, serial, _, _ = robot
    assert post(port, "/mission", MISSION) == (200, {"error": None})
    wait_until(lambda: serial.commands)
    wait_until(lambda: state_of(port)["returned_home"] and state_of(port)["state"] == "IDLE")
    state = state_of(port)
    assert state["destination_reached"] and state["position"] == [0, 1]
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)

    # Le robot accepte une nouvelle livraison, et le résultat précédent ne reste pas affiché.
    assert post(port, "/mission", {**MISSION, "destination": [0, 3]})[0] == 200
    wait_until(lambda: state_of(port)["destination"] == [0, 3])
    assert not state_of(port)["returned_home"]


def test_arreter_coupe_les_moteurs_immediatement(robot: tuple) -> None:
    port, serial, _, _ = robot
    far = {"start": [0, 0], "destination": [4, 4], "heading": "SOUTH"}
    assert post(port, "/mission", far)[0] == 200
    wait_until(lambda: any(c != (0.0, 0.0, 0.0, 0.0) for c in serial.commands))

    assert post(port, "/stop", {})[0] == 200
    wait_until(lambda: state_of(port)["state"] == "IDLE")
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)
    count = len(serial.commands)
    time.sleep(0.05)
    assert len(serial.commands) == count, "plus aucune commande moteur après l'arrêt"
    assert not state_of(port)["returned_home"]

    # L'arrêt ne doit pas empêcher la mission suivante.
    assert post(port, "/mission", MISSION)[0] == 200
    wait_until(lambda: state_of(port)["returned_home"] and state_of(port)["state"] == "IDLE")


def test_deuxieme_ordre_refuse_pendant_une_mission(robot: tuple) -> None:
    port, _, _, _ = robot
    assert post(port, "/mission", {"start": [0, 0], "destination": [4, 4], "heading": "SOUTH"})[0] == 200
    assert post(port, "/mission", MISSION)[0] == 409
    post(port, "/stop", {})


def test_ordre_invalide_ne_lance_rien(robot: tuple) -> None:
    port, serial, connection, _ = robot
    status, body = post(port, "/mission", {**MISSION, "destination": [0, 1]})
    assert status == 400 and "différents" in body["error"]
    steps = connection.steps
    wait_until(lambda: connection.steps > steps + 20)
    assert serial.commands == []


def test_autre_site_ne_peut_pas_demarrer_le_robot(robot: tuple) -> None:
    """Un formulaire d'un autre site ne peut envoyer que du texte, sans passer de contrôle CORS."""
    port, serial, connection, _ = robot
    assert post(port, "/mission", MISSION, content_type="text/plain")[0] == 415
    steps = connection.steps
    wait_until(lambda: connection.steps > steps + 20)
    assert serial.commands == []


def test_arret_au_repos_n_annule_pas_la_mission_suivante(robot: tuple) -> None:
    port, _, _, _ = robot
    assert post(port, "/stop", {})[0] == 200
    assert post(port, "/mission", MISSION)[0] == 200
    wait_until(lambda: state_of(port)["returned_home"])


def test_huit_tourne_jusqu_a_l_arret_puis_livraison_possible(robot: tuple) -> None:
    port, serial, connection, _ = robot
    assert post(port, "/mission", {"mode": "HUIT"}) == (200, {"error": None})
    wait_until(lambda: state_of(port)["mode"] == "HUIT")
    wait_until(lambda: any(c != (0.0, 0.0, 0.0, 0.0) for c in serial.commands))

    # FakeProcessor voit un croisement toutes les 40 images : le robot doit en passer plusieurs.
    steps = connection.steps
    wait_until(lambda: connection.steps > steps + 200)
    assert state_of(port)["mode"] == "HUIT" and state_of(port)["state"] != "IDLE"
    assert post(port, "/mission", MISSION)[0] == 409, "un seul ordre à la fois"

    assert post(port, "/stop", {})[0] == 200
    wait_until(lambda: state_of(port)["state"] == "IDLE")
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)

    assert post(port, "/mission", MISSION)[0] == 200
    wait_until(lambda: state_of(port)["returned_home"] and state_of(port)["state"] == "IDLE")


def test_livraison_precedente_oubliee_apres_un_huit(robot: tuple) -> None:
    port, _, _, _ = robot
    assert post(port, "/mission", MISSION)[0] == 200
    wait_until(lambda: state_of(port)["returned_home"] and state_of(port)["state"] == "IDLE")

    assert post(port, "/mission", {"mode": "HUIT"})[0] == 200
    wait_until(lambda: state_of(port)["mode"] == "HUIT")
    assert post(port, "/stop", {})[0] == 200
    wait_until(lambda: state_of(port)["state"] == "IDLE")
    state = state_of(port)
    assert state["start"] is None and not state["returned_home"] and state["route"] == []


@pytest.mark.parametrize("order", [MISSION, {"mode": "HUIT"}])
def test_ligne_perdue_signalee_a_l_interface_puis_effacee(order: dict) -> None:
    visible = {"line": True}
    server = TelemetryServer(port=0)
    serial, connection = FakeSerial(), PacedConnection()
    processor = LineScriptProcessor(lambda i: visible["line"])
    thread = threading.Thread(target=serve_missions, args=(connection, serial, server, processor))
    thread.start()
    port = server._server.server_address[1]
    try:
        wait_until(lambda: connection.steps > 2)
        assert state_of(port)["line_lost"] is False

        assert post(port, "/mission", order)[0] == 200
        wait_until(lambda: state_of(port)["state"] != "IDLE")
        visible["line"] = False
        wait_until(lambda: state_of(port)["state"] == "IDLE")
        assert state_of(port)["line_lost"] is True
        assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)

        # Le robot reste arrêté, même si la ligne revient, et une nouvelle mission efface l'alerte.
        count = len(serial.commands)
        visible["line"] = True
        steps = connection.steps
        wait_until(lambda: connection.steps > steps + 50)
        assert len(serial.commands) == count
        assert state_of(port)["line_lost"] is True

        assert post(port, "/mission", order)[0] == 200
        wait_until(lambda: state_of(port)["state"] != "IDLE")
        assert state_of(port)["line_lost"] is False
    finally:
        server._request_stop()
        connection.finished = True
        thread.join(10)
        server.close()
    assert not thread.is_alive()


# --- Point d'entrée --------------------------------------------------------------------------

def _patch_main(monkeypatch: pytest.MonkeyPatch) -> Tuple[mock.MagicMock, mock.MagicMock, mock.MagicMock]:
    telemetry, dashboard, opened = mock.MagicMock(), mock.MagicMock(), mock.MagicMock()
    monkeypatch.setattr(EiNav, "start_telemetry", lambda: telemetry)
    monkeypatch.setattr(EiNav, "start_dashboard", lambda: dashboard)
    monkeypatch.setattr(EiNav, "open_interface", opened)
    return telemetry, dashboard, opened


def test_main_ferme_tout_si_le_materiel_est_absent(monkeypatch: pytest.MonkeyPatch) -> None:
    telemetry, dashboard, opened = _patch_main(monkeypatch)

    def no_arduino(simulation: bool) -> None:
        raise OSError("/dev/ttyACM0 introuvable")

    monkeypatch.setattr(EiNav, "create_environment", no_arduino)
    with pytest.raises(OSError):
        EiNav.main()
    opened.assert_called_once_with(dashboard)
    telemetry.close.assert_called_once()
    dashboard.terminate.assert_called_once()


def test_main_ouvre_l_interface_avant_le_robot_et_ferme_sur_ctrl_c(monkeypatch: pytest.MonkeyPatch) -> None:
    telemetry, dashboard, opened = _patch_main(monkeypatch)
    connection, serial = mock.MagicMock(), mock.MagicMock()

    def environment(simulation: bool) -> Tuple[mock.MagicMock, mock.MagicMock]:
        opened.assert_called_once_with(dashboard)
        return connection, serial

    def ctrl_c(*args: object) -> None:
        raise KeyboardInterrupt

    monkeypatch.setattr(EiNav, "create_environment", environment)
    monkeypatch.setattr(EiNav, "serve_missions", ctrl_c)
    EiNav.main()
    serial.disconnect.assert_called_once()
    connection.close.assert_called_once()
    telemetry.close.assert_called_once()
    dashboard.terminate.assert_called_once()


def test_main_s_arrete_si_l_interface_est_indisponible(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(EiNav, "start_telemetry", lambda: None)
    create = mock.MagicMock()
    monkeypatch.setattr(EiNav, "create_environment", create)
    with pytest.raises(SystemExit):
        EiNav.main()
    create.assert_not_called()
