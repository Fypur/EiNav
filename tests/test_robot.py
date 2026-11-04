"""Tests de la couche robot, sans moteurs ni caméra."""
from __future__ import annotations

import json
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import List, Optional, Tuple
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
from robot.control import Control  # noqa: E402
from robot.state_machine import get_turn_state, run_state_machine  # noqa: E402
from robot.states import Mode, RobotState  # noqa: E402
from robot.telemetry import TelemetryServer, make_status  # noqa: E402
from vision.image_processor import ImageProcessor, LineDetection  # noqa: E402


class FakeSerial:
    def __init__(self) -> None:
        self.commands: List[Tuple[float, float, float, float]] = []

    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        self.commands.append((fl, fr, rr, rl))

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
    assert speeds[1] < speeds[2] < control.base_speed
    assert speeds[-1] == control.base_speed
    # Écart à la consigne divisé par un facteur constant : c'est une exponentielle.
    ratios = [(control.base_speed - speeds[i + 1]) / (control.base_speed - speeds[i]) for i in range(1, 6)]
    assert ratios == pytest.approx([np.exp(-dt / control.time_constant)] * 5)

    t = 40 * dt
    stopped = False
    for i in range(40):
        stopped = control.stop(t + i * dt)
        if stopped:
            break
    assert stopped
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)


@pytest.mark.parametrize("quarters", [1, 2])
def test_rotation_conserve_angle_malgre_les_rampes(quarters: int) -> None:
    control, serial = _make_control()
    dt = 0.01
    t = 0.0
    while not control.turn(t, False, 0.0, quarter_turns=quarters):
        t += dt
    integral = sum(c[0] for c in serial.commands) * dt
    assert integral == pytest.approx(control.turn_speed * control.turn_duration * quarters, rel=0.02)
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
    assert distance == pytest.approx(control.forward_speed * 0.9, rel=0.1)


def test_correction_de_direction_non_filtree() -> None:
    control, serial = _make_control()
    control.aim(0.0, 1.0)
    fl, fr, _, _ = serial.commands[-1]
    assert fl - fr == pytest.approx(2 * control.kp)


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

    # Le demi-tour a lieu sur la destination, juste après la levée du flag.
    first_u_turn = states.index(RobotState.TURNING_180.name)
    assert recorder.statuses[first_u_turn].position == (2, 2)
    assert recorder.statuses[first_u_turn].destination_reached
    assert recorder.statuses[first_u_turn - 1].destination_reached is False
    assert recorder.statuses[-1].state == RobotState.STOPPED.name
    assert recorder.statuses[-1].returned_home


def test_image_perdue_ne_fait_pas_planter() -> None:
    connection = FakeConnection(drop_first=5)
    mission = run_state_machine(connection, FakeSerial(), (0, 0), (0, 0), Direction.SOUTH,  # type: ignore[arg-type]
                                processor=FakeProcessor())
    assert mission.returned_home


def test_chemin_impossible() -> None:
    with pytest.raises(ValueError):
        run_state_machine(FakeConnection(), FakeSerial(), (0, 0), (9, 9), Direction.SOUTH,  # type: ignore[arg-type]
                          processor=FakeProcessor())


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

        mission = Mission(Navigation().generate_path(Grid(4), (0, 1), (1, 1), Direction.SOUTH))
        mission.on_intersection()
        server.publish(make_status(Mode.GRID, RobotState.FOLLOWING, 5, mission, Direction.EAST, True, False))
        with urllib.request.urlopen(base + "/state.json") as response:
            state = json.loads(response.read())
        assert state["state"] == "FOLLOWING"
        assert state["position"] == [0, 1]
        assert state["next_node"] == [1, 1]
        assert state["grid_nodes"] == 5

        with urllib.request.urlopen(base + "/") as response:
            assert b"Robot en direct" in response.read()
    finally:
        server.close()


# --- Point d'entrée --------------------------------------------------------------------------

def test_main_ferme_tout_si_le_materiel_est_absent(monkeypatch: pytest.MonkeyPatch) -> None:
    telemetry, dashboard = mock.MagicMock(), mock.MagicMock()
    monkeypatch.setattr(EiNav, "start_telemetry", lambda: telemetry)
    monkeypatch.setattr(EiNav, "start_dashboard", lambda: dashboard)

    def no_arduino(simulation: bool) -> None:
        raise OSError("/dev/ttyACM0 introuvable")

    monkeypatch.setattr(EiNav, "create_environment", no_arduino)
    with pytest.raises(OSError):
        EiNav.main()
    telemetry.close.assert_called_once()
    dashboard.terminate.assert_called_once()


def test_main_garde_le_tableau_de_bord_jusqu_a_ctrl_c(monkeypatch: pytest.MonkeyPatch) -> None:
    telemetry, dashboard, connection, serial = (mock.MagicMock() for _ in range(4))
    monkeypatch.setattr(EiNav, "start_telemetry", lambda: telemetry)
    monkeypatch.setattr(EiNav, "start_dashboard", lambda: dashboard)
    monkeypatch.setattr(EiNav, "create_environment", lambda simulation: (connection, serial))
    monkeypatch.setattr(EiNav, "run_state_machine", lambda *args, **kwargs: None)

    def ctrl_c(_: float) -> None:
        telemetry.close.assert_not_called()  # encore consultable après la mission
        raise KeyboardInterrupt

    monkeypatch.setattr(EiNav.time, "sleep", ctrl_c)
    EiNav.main()
    serial.disconnect.assert_called_once()
    connection.close.assert_called_once()
    telemetry.close.assert_called_once()
    dashboard.terminate.assert_called_once()
