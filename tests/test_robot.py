"""Tests de la couche robot, sans moteurs ni caméra."""
from __future__ import annotations

import json
import sys
import threading
import time
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
from robot.state_machine import get_turn_state, run_state_machine, serve_missions  # noqa: E402
from robot.states import Mode, RobotState  # noqa: E402
from robot.telemetry import MissionRequest, TelemetryServer, make_status, parse_mission  # noqa: E402
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
            assert b"Mode livraison" in response.read()
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
    assert parse_mission(json.dumps(MISSION).encode(), 5) == MissionRequest((0, 1), (1, 1), Direction.SOUTH)


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
