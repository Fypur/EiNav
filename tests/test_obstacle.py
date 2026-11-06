"""Tests du freinage devant un obstacle (capteur ultrason)."""
from __future__ import annotations

import json
import math
import struct
import threading
import time
import urllib.request
from typing import Any, Callable, List, Optional, Tuple
from unittest import mock

import numpy as np
import pytest

from connections.arduino import Arduino
from connections.base import RobotConnection
from connections.serialpi import Serial
from connections.utilitaries import retrCmdli
from planning.direction import Direction
from robot import utils
from robot.control import Control
from robot.obstacle import ObstacleGuard, ObstacleState
from robot.state_machine import run_figure_eight, run_state_machine, serve_missions
from robot.states import RobotState
from robot.telemetry import TelemetryServer
from vision.image_processor import ImageProcessor, LineDetection


# Lecture du capteur

class FakePort:
    def __init__(self, reply: bytes) -> None:
        self.reply = reply
        self.written = b""
        self.flushed = False

    def write(self, data: bytes) -> None:
        self.written += data

    def read(self, size: int) -> bytes:
        return self.reply[:size]

    def reset_input_buffer(self) -> None:
        self.flushed = True


def test_lecture_de_la_reponse_binaire_de_l_arduino() -> None:
    """Ret2li en binaire (commode == 2) : long millis, int distance, int nul, en little endian."""
    arduino = Arduino.__new__(Arduino)
    arduino.serialport = FakePort(struct.pack("<lhh", 123456, 42, 0))  # type: ignore[assignment]
    assert arduino.GetDistance() == 42
    assert arduino.serialport.written == b"S"  # type: ignore[attr-defined]


@pytest.mark.parametrize("reply", [b"", b"\x01\x02\x03"])
def test_reponse_absente_ou_tronquee(reply: bytes) -> None:
    port = FakePort(reply)
    assert retrCmdli(port, b"S") is None  # type: ignore[arg-type]
    assert port.flushed, "une réponse tardive ne doit pas être lue comme un acquittement"


def test_simulation_ne_voit_jamais_d_obstacle() -> None:
    serial = Serial(robot=mock.MagicMock())
    assert serial.get_distance_cm() == 200


# Loi de vitesse

def caps(*readings: Optional[int]) -> float:
    guard = ObstacleGuard()
    cap = math.inf
    for reading in readings:
        cap = guard.update(reading)
    return cap


@pytest.mark.parametrize("distance", [60, 61, 200])
def test_aucun_ralentissement_a_partir_de_60_cm(distance: int) -> None:
    assert caps(distance) == math.inf


def test_ralentit_des_59_cm_puis_proportionnellement() -> None:
    assert caps(59) == pytest.approx(utils.BASE_SPEED * 39 / 40)
    assert caps(40) == pytest.approx(utils.BASE_SPEED * 0.5)
    assert caps(59) > caps(50) > caps(40) > caps(30)


def test_vitesse_minimale_avant_l_arret_pour_ne_pas_caler() -> None:
    assert caps(21) == utils.FORWARD_SPEED


@pytest.mark.parametrize("distance", [20, 19, 4])
def test_arret_a_20_cm_et_moins(distance: int) -> None:
    assert caps(distance) == 0.0


@pytest.mark.parametrize("reading", [None, 0])
def test_sans_mesure_le_capteur_est_ignore(reading: Optional[int]) -> None:
    guard = ObstacleGuard()
    assert guard.update(reading) == math.inf
    assert guard.status == (ObstacleState.NO_MEASURE, None)


@pytest.mark.parametrize("reading", [None, 0])
def test_mesure_absente_isolee_n_efface_pas_l_obstacle(reading: Optional[int]) -> None:
    assert caps(20, reading) == 0.0
    assert caps(30, reading) == caps(30)
    assert caps(30, reading, reading) == caps(30)


@pytest.mark.parametrize("nearby", [20, 30])
def test_capteur_devenu_muet_est_ignore_meme_pres_d_un_obstacle(nearby: int) -> None:
    assert caps(nearby, None, None, None) == math.inf


def test_mesures_revenues_reprennent_effet() -> None:
    assert caps(None, None, None, 20) == 0.0
    assert caps(None, None, None, 40) == caps(40)


def test_echo_manque_ne_relance_pas_le_robot() -> None:
    """Un écho raté donne 200 cm, ça ne doit pas effacer un obstacle proche."""
    assert caps(30, 200) == caps(30)
    assert caps(30, 200, 200) == caps(30)
    assert caps(30, 200, 200, 200) == math.inf, "repart une fois l'obstacle réellement parti"


def test_mesure_proche_prise_en_compte_immediatement() -> None:
    assert caps(200, 200, 20) == 0.0


def test_mesure_oscillante_a_20_cm_ne_fait_pas_avancer_par_a_coups() -> None:
    assert all(caps(*([20, 21] * n)) == 0.0 for n in range(1, 10))
    assert caps(20, 24, 24, 24) == 0.0, "toujours arrêté en dessous de la distance de reprise"
    assert caps(20, 25, 25, 25) > 0.0


def test_etats_pour_l_interface() -> None:
    guard = ObstacleGuard()
    guard.update(200)
    assert guard.status == (ObstacleState.CLEAR, 200)
    guard.update(45)
    assert guard.status == (ObstacleState.SLOWING, 45)
    guard.update(18)
    assert guard.status == (ObstacleState.BLOCKED, 18)


# Commande des moteurs

class WheelRecorder:
    def __init__(self) -> None:
        self.commands: List[Tuple[float, float, float, float]] = []

    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        self.commands.append((fl, fr, rr, rl))


def _control() -> Tuple[Control, WheelRecorder]:
    serial = WheelRecorder()
    control = Control(serial)  # type: ignore[arg-type]
    for i in range(100):
        control.aim(i * 0.05, 0.0)
    return control, serial


def test_le_plafond_freine_sans_retard() -> None:
    control, serial = _control()
    assert serial.commands[-1][0] == utils.BASE_SPEED
    control.set_speed_cap(0.2)
    control.aim(5.0, 0.0)
    assert serial.commands[-1][0] == pytest.approx(0.2)
    control.set_speed_cap(0.0)
    control.aim(5.05, 0.0)
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)


def test_a_l_arret_la_correction_ne_fait_pas_pivoter() -> None:
    control, serial = _control()
    control.set_speed_cap(0.0)
    control.aim(5.0, 1.0)
    assert serial.commands[-1] == (0.0, 0.0, 0.0, 0.0)


def test_correction_proportionnelle_a_la_vitesse() -> None:
    """Même trajectoire : le rapport entre correction et vitesse ne change pas."""
    control, serial = _control()
    control.aim(5.0, 1.0)
    fl, fr, _, _ = serial.commands[-1]
    full_ratio = (fl - fr) / (fl + fr)
    control.set_speed_cap(utils.BASE_SPEED / 2)
    control.aim(5.05, 1.0)
    fl, fr, _, _ = serial.commands[-1]
    assert (fl - fr) / (fl + fr) == pytest.approx(full_ratio)
    assert fl + fr == pytest.approx(utils.BASE_SPEED)


def test_repart_en_douceur_apres_l_obstacle() -> None:
    control, serial = _control()
    control.set_speed_cap(0.0)
    control.aim(5.0, 0.0)
    control.set_speed_cap(math.inf)
    control.aim(5.05, 0.0)
    assert 0.0 < serial.commands[-1][0] < utils.BASE_SPEED


def test_rotation_sur_place_non_bloquee() -> None:
    """Un virage ne rapproche pas de l'obstacle et il faut pouvoir s'en écarter."""
    control, serial = _control()
    control.set_speed_cap(0.0)
    t = 5.0
    for _ in range(40):
        control.turn(t, False, 5.0)
        t += 0.05
    assert serial.commands[-1] != (0.0, 0.0, 0.0, 0.0) or control.is_stopped
    assert max(c[0] for c in serial.commands[-40:]) > 0.5 * utils.TURN_SPEED


def test_temps_de_conduite_fige_a_l_arret() -> None:
    control, _ = _control()
    before = control.drive_time
    control.set_speed_cap(0.0)
    control.aim(5.0, 0.0)
    control.aim(9.0, 0.0)
    assert control.drive_time == before


# Comportement complet, sur un robot simulé qui avance vraiment

class Robot(RobotConnection):
    """Robot sur une ligne droite, sa position dépend des vitesses de roue envoyées."""

    def __init__(self, cm_per_second_at_full_speed: float, obstacle_at: float = 150.0, dt: float = 0.064,
                 max_steps: int = 2000) -> None:
        self.gain = cm_per_second_at_full_speed
        self.obstacle_at = obstacle_at
        self.dt = dt
        self.t = 0.0
        self.steps = 0
        self.max_steps = max_steps
        self.position = 0.0
        self.wheel = 0.0
        self.speeds: List[Tuple[float, float]] = []  # (distance réelle à l'obstacle, vitesse)

    @property
    def distance(self) -> float:
        return self.obstacle_at - self.position

    # Connexion
    def get_time(self) -> float:
        return self.t

    def step(self) -> bool:
        self.t += self.dt
        self.steps += 1
        self.position += self.wheel * self.gain * self.dt
        return self.steps < self.max_steps

    def get_frame(self) -> np.ndarray:
        return np.zeros((128, 160, 3), np.uint8)

    def publish_frame(self, frame: np.ndarray) -> None:
        pass

    # Liaison série
    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        self.wheel = (fl + fr) / 2
        self.speeds.append((self.distance, self.wheel))

    def get_distance_cm(self) -> Optional[int]:
        return min(200, int(self.distance))  # l'Arduino tronque au cm


class LineAhead(ImageProcessor):
    def __init__(self, visible: Callable[[int], bool] = lambda i: True) -> None:
        self.visible = visible
        self.calls = 0

    def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
        self.calls += 1
        cx = 80 if self.visible(self.calls) else None
        return LineDetection(cx, False, 160, 76, np.zeros((52, 160), np.uint8))


class Recorder:
    stop_requested = False

    def __init__(self) -> None:
        self.statuses: list = []

    def publish(self, status: Any) -> None:
        self.statuses.append(status)


def _drive_to_obstacle(gain: float, base_speed: float = utils.BASE_SPEED, **kwargs: Any) -> Tuple[Robot, Recorder]:
    robot, recorder = Robot(gain, **kwargs), Recorder()
    with mock.patch.object(utils, "BASE_SPEED", base_speed):
        line_lost = run_figure_eight(robot, robot, LineAhead(), recorder)  # type: ignore[arg-type]
    assert not line_lost
    return robot, recorder


@pytest.mark.parametrize("gain", [40.0, 80.0, 150.0])
@pytest.mark.parametrize("base_speed", [0.3, 0.4, 0.8])
def test_s_arrete_a_20_cm_de_l_obstacle(gain: float, base_speed: float) -> None:
    robot, _ = _drive_to_obstacle(gain, base_speed)
    assert robot.wheel == 0.0
    # La mesure est tronquée (20 cm lus pour 20,x cm) et il reste une période de boucle à vitesse lente
    overshoot = utils.FORWARD_SPEED * gain * robot.dt
    assert 20.0 - overshoot <= robot.distance < 21.0


def test_vitesse_de_croisiere_jusqu_a_60_cm_puis_decroissante() -> None:
    robot, _ = _drive_to_obstacle(80.0)
    far = [speed for distance, speed in robot.speeds[20:] if distance > 61.0]
    assert far and all(speed == pytest.approx(utils.BASE_SPEED, abs=0.02) for speed in far)

    # int(59,9) = 59, donc le freinage commence dès qu'on passe sous 60 cm
    braking = [(d, s) for d, s in robot.speeds if 21.0 < d < 59.0]
    assert braking
    assert all(s < utils.BASE_SPEED for _, s in braking)
    speeds = [s for _, s in braking]
    assert speeds == sorted(speeds, reverse=True)
    assert speeds[-1] == pytest.approx(utils.FORWARD_SPEED)


def test_le_robot_ne_redemarre_pas_tant_que_l_obstacle_reste() -> None:
    robot, recorder = _drive_to_obstacle(80.0)
    stopped_at = robot.distance
    assert recorder.statuses[-1].obstacle_state == "BLOCKED"
    assert robot.distance == stopped_at
    assert all(s == 0.0 for _, s in robot.speeds[-100:])


def test_repart_quand_l_obstacle_s_en_va() -> None:
    class Leaving(Robot):
        def step(self) -> bool:
            if self.t > 20.0:
                self.obstacle_at = 10000.0
            return super().step()

    robot = Leaving(80.0, max_steps=500)
    run_figure_eight(robot, robot, LineAhead(), Recorder())  # type: ignore[arg-type]
    assert robot.position > 200.0


@pytest.mark.parametrize("reading", [None, 0])
def test_mission_poursuivie_sans_mesure_du_capteur(reading: Optional[int]) -> None:
    class Mute(Robot):
        def get_distance_cm(self) -> Optional[int]:
            return reading

    robot, recorder = Mute(80.0, obstacle_at=20.0, max_steps=200), Recorder()
    run_figure_eight(robot, robot, LineAhead(), recorder)  # type: ignore[arg-type]
    assert robot.position > 100.0, "le robot roule normalement"
    assert max(s for _, s in robot.speeds) == pytest.approx(utils.BASE_SPEED, abs=0.02)
    assert recorder.statuses[-1].obstacle_state == "NO_MEASURE"


def test_livraison_terminee_sans_mesure_du_capteur() -> None:
    class Mute(Robot):
        def get_distance_cm(self) -> Optional[int]:
            return None

    class CrossingEvery40(LineAhead):
        def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
            detection = super().process_frame(image_bgr)
            return detection._replace(is_intersection=self.calls % 40 < 3)

    robot = Mute(80.0)
    mission = run_state_machine(robot, robot, (0, 0), (0, 2), Direction.SOUTH,  # type: ignore[arg-type]
                                processor=CrossingEvery40())
    assert mission.destination_reached and mission.returned_home


def test_pas_de_ligne_perdue_quand_bloque_devant_un_obstacle() -> None:
    """Plusieurs secondes à l'arrêt sans voir la ligne, ce n'est pas une sortie de piste."""
    robot = Robot(80.0, obstacle_at=20.0, max_steps=200)
    line_lost = run_figure_eight(robot, robot, LineAhead(lambda i: False), Recorder())  # type: ignore[arg-type]
    assert line_lost is False
    assert robot.position == 0.0


def test_ligne_perdue_toujours_detectee_en_roulant_lentement() -> None:
    robot = Robot(80.0, obstacle_at=45.0, max_steps=200)
    line_lost = run_figure_eight(robot, robot, LineAhead(lambda i: False), Recorder())  # type: ignore[arg-type]
    assert line_lost is True


class CrossingAtFrame(LineAhead):
    def __init__(self, frame: int) -> None:
        super().__init__()
        self.frame = frame

    def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
        detection = super().process_frame(image_bgr)
        return detection._replace(is_intersection=self.calls in (self.frame, self.frame + 1))


def test_approche_d_un_croisement_figee_pendant_le_blocage() -> None:
    """Le robot bloqué n'avance pas vers le centre, il finit l'approche une fois libéré."""
    class Blocking(Robot):
        def get_distance_cm(self) -> Optional[int]:
            return 20 if 12 <= self.steps < 24 else 200  # moins que OBSTACLE_CONFIRM_SECONDS

    robot, recorder = Blocking(80.0, max_steps=140), Recorder()
    run_state_machine(robot, robot, (0, 0), (0, 2), Direction.SOUTH,  # type: ignore[arg-type]
                      processor=CrossingAtFrame(10), telemetry=recorder)  # type: ignore[arg-type]
    states = [s.state for s in recorder.statuses]
    assert RobotState.APPROACHING_INTERSECTION.name in states
    blocked = [s.state for s in recorder.statuses if s.obstacle_state == "BLOCKED"]
    assert blocked and set(blocked) == {RobotState.APPROACHING_INTERSECTION.name}, \
        "reste en approche tant que l'obstacle est là, sans déclarer le centre atteint"


def test_virage_au_depart_malgre_un_obstacle_proche() -> None:
    robot, recorder = Robot(80.0, obstacle_at=15.0, max_steps=120), Recorder()
    run_state_machine(robot, robot, (0, 0), (0, 2), Direction.NORTH,  # type: ignore[arg-type]
                      processor=LineAhead(), telemetry=recorder)  # type: ignore[arg-type]
    states = [s.state for s in recorder.statuses]
    assert RobotState.TURNING_180.name in states
    assert states[-1] == RobotState.FOLLOWING.name, "le demi-tour se termine"
    assert robot.position == 0.0 or abs(robot.position) < 1e-9, "aucune avance vers l'obstacle"


# Interface

def test_etat_du_capteur_publie_au_repos_et_arret_interface() -> None:
    class Idle(Robot):
        def step(self) -> bool:
            time.sleep(0.001)
            return super().step() and not self.finished

        finished = False

    robot = Idle(80.0, obstacle_at=35.0, max_steps=10 ** 9)
    server = TelemetryServer(port=0)
    port = server._server.server_address[1]
    thread = threading.Thread(target=serve_missions, args=(robot, robot, server, LineAhead()))
    thread.start()

    def state() -> dict:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/state.json") as response:
            return json.loads(response.read())

    try:
        deadline = time.monotonic() + 5
        while robot.steps < 5 and time.monotonic() < deadline:
            time.sleep(0.005)
        idle = state()
        assert idle["state"] == "IDLE"
        assert idle["obstacle_state"] == "SLOWING" and idle["obstacle_cm"] == 35
        assert robot.wheel == 0.0, "lire le capteur au repos ne fait pas bouger le robot"
    finally:
        robot.finished = True
        thread.join(10)
        server.close()
    assert not thread.is_alive()
