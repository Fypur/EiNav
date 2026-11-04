"""Tests des réglages modifiables depuis l'interface."""
from __future__ import annotations

import json
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import pytest

from connections import utils as connections_utils
from robot import tuning
from robot import utils as robot_utils
from robot.control import Control
from robot.telemetry import TelemetryServer
from vision import utils as vision_utils
from vision.image_processor import ImageProcessor

ROOT = Path(__file__).resolve().parents[1]


class FakeSerial:
    def __init__(self) -> None:
        self.commands: List[Tuple[float, float, float, float]] = []

    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        self.commands.append((fl, fr, rr, rl))


def settled(control: Control, serial: FakeSerial, error: float = 0.0) -> Tuple[float, ...]:
    for i in range(400):
        control.aim(i * 0.02, error)
    return serial.commands[-1]


def values() -> Dict[str, Any]:
    return {entry["key"]: entry["value"] for entry in tuning.describe()}


# --- Valeurs ---------------------------------------------------------------------------------

def test_les_defauts_respectent_leurs_propres_bornes() -> None:
    for entry in tuning.describe():
        assert entry["min"] <= entry["default"] <= entry["max"], entry["key"]


def test_chaque_reglage_existe_dans_son_module() -> None:
    for setting in tuning.SETTINGS:
        assert hasattr(setting.module, setting.key)


@pytest.mark.parametrize("key,value", [
    ("KP", 1.5), ("KP", -0.1), ("KP", "0.2"), ("KP", True), ("KP", None), ("KP", float("nan")), ("KP", float("inf")),
    ("KP", 10 ** 400),  # json.loads accepte les entiers géants, que math.isfinite refuse par une exception
    ("FORWARD_SPEED", 0.0),  # diviseur dans Control.approach_duration
    ("RAMP_TIME_CONSTANT", 0.0),  # diviseur dans Control._drive
    ("BINARY_THRESHOLD", 300), ("BINARY_THRESHOLD", 140.5), ("TIME_STEP_MS", 0),
    ("INCONNU", 1), ("SERIAL_PORT", 1), ("__class__", 1),
])
def test_valeur_invalide_refusee(key: str, value: object) -> None:
    before = values()
    with pytest.raises(ValueError):
        tuning.update({key: value})
    assert values() == before


def test_changement_tout_ou_rien() -> None:
    before = robot_utils.BASE_SPEED
    with pytest.raises(ValueError):
        tuning.update({"BASE_SPEED": 0.5, "KP": 99})
    assert robot_utils.BASE_SPEED == before


# --- Effet réel sur le robot -----------------------------------------------------------------

def test_vitesse_appliquee_pendant_la_mission() -> None:
    serial = FakeSerial()
    control = Control(serial)  # type: ignore[arg-type]
    assert settled(control, serial)[0] == robot_utils.BASE_SPEED
    tuning.update({"BASE_SPEED": 0.7})
    for i in range(400):
        control.aim(10 + i * 0.02, 0.0)
    assert serial.commands[-1][0] == 0.7


def test_kp_change_la_correction() -> None:
    tuning.update({"KP": 0.3})
    serial = FakeSerial()
    fl, fr, _, _ = settled(Control(serial), serial, error=1.0)  # type: ignore[arg-type]
    assert fl - fr == pytest.approx(0.6)


def test_duree_de_virage_appliquee() -> None:
    tuning.update({"TURN_DURATION": 0.5})
    serial = FakeSerial()
    control = Control(serial)  # type: ignore[arg-type]
    assert control.turn(0.4, False, 0.0) is False
    t = 0.5
    while not control.turn(t, False, 0.0):
        t += 0.01
    assert t < 2.0


def test_approche_utilise_la_vitesse_d_approche() -> None:
    serial = FakeSerial()
    control = Control(serial)  # type: ignore[arg-type]
    tuning.update({"FORWARD_SPEED": 0.3, "RAMP_TIME_CONSTANT": 0.2})
    settled(control, serial)
    expected = 0.9 - robot_utils.BASE_SPEED * 0.2 / 0.3
    assert control.approach_duration(0.9) == pytest.approx(expected)


def test_roues_jamais_au_dela_du_maximum() -> None:
    """Au-delà de 255, analogWrite tronque sur 8 bits : la roue ralentirait au lieu d'accélérer."""
    tuning.update({"BASE_SPEED": 1.0, "KP": 1.0})
    serial = FakeSerial()
    fl, fr, rr, rl = settled(Control(serial), serial, error=1.0)  # type: ignore[arg-type]
    assert fl == rl == 1.0 and fr == rr == 0.0
    tuning.update({"TURN_SPEED": 1.0})
    control = Control(serial)  # type: ignore[arg-type]
    for i in range(100):
        control.turn(i * 0.02, False, 0.0)
    assert max(abs(v) for c in serial.commands for v in c) <= 1.0


def _cross() -> np.ndarray:
    frame = np.zeros((128, 160, 3), np.uint8)
    frame[:, 70:90] = 180
    frame[95:110, :] = 180
    return frame


def test_seuil_et_zone_appliques_a_la_vision() -> None:
    processor = ImageProcessor()
    assert processor.process_frame(_cross()).cx is not None
    tuning.update({"BINARY_THRESHOLD": 200})
    assert processor.process_frame(_cross()).cx is None
    tuning.update({"BINARY_THRESHOLD": 140, "ROI_TOP_RATIO": 0.3})
    assert processor.process_frame(_cross()).roi_top == 38
    tuning.update({"MIN_LINE_AREA": 2000, "ROI_TOP_RATIO": 0.9})
    assert processor.process_frame(_cross()).cx is None


@pytest.mark.parametrize("ratio", [0.1, 0.9])
def test_vision_supporte_les_bornes_de_zone(ratio: float) -> None:
    tuning.update({"ROI_TOP_RATIO": ratio})
    processor = ImageProcessor()
    processor.annotate(_cross(), processor.process_frame(_cross()))


# --- Conservation ----------------------------------------------------------------------------

def test_conserve_au_redemarrage_et_seulement_les_differences() -> None:
    tuning.update({"KP": 0.33, "TIME_STEP_MS": 20, "BASE_SPEED": tuning._DEFAULTS["BASE_SPEED"]})
    assert json.loads(tuning.SETTINGS_FILE.read_text(encoding="utf-8")) == {"KP": 0.33, "TIME_STEP_MS": 20}

    robot_utils.KP = tuning._DEFAULTS["KP"]
    connections_utils.TIME_STEP_MS = tuning._DEFAULTS["TIME_STEP_MS"]
    tuning.load()
    assert robot_utils.KP == 0.33
    assert connections_utils.TIME_STEP_MS == 20


def test_reinitialisation() -> None:
    tuning.update({"KP": 0.33, "BASE_SPEED": 0.7, "BINARY_THRESHOLD": 100})
    tuning.reset(["KP"])
    assert robot_utils.KP == tuning._DEFAULTS["KP"] and robot_utils.BASE_SPEED == 0.7
    tuning.reset()
    assert robot_utils.BASE_SPEED == tuning._DEFAULTS["BASE_SPEED"]
    assert vision_utils.BINARY_THRESHOLD == tuning._DEFAULTS["BINARY_THRESHOLD"]
    assert json.loads(tuning.SETTINGS_FILE.read_text(encoding="utf-8")) == {}


@pytest.mark.parametrize("content", ["{pas du json", "[1, 2]", "", '"texte"'])
def test_fichier_corrompu_ne_bloque_pas_le_robot(content: str) -> None:
    tuning.SETTINGS_FILE.write_text(content, encoding="utf-8")
    before = values()
    tuning.load()
    assert values() == before


def test_valeur_invalide_dans_le_fichier_ignoree_les_autres_gardees() -> None:
    tuning.SETTINGS_FILE.write_text(json.dumps({"KP": 99, "INCONNU": 1, "BASE_SPEED": 0.7}), encoding="utf-8")
    tuning.load()
    assert robot_utils.KP == tuning._DEFAULTS["KP"]
    assert robot_utils.BASE_SPEED == 0.7


def test_chargement_ne_reecrit_pas_le_fichier() -> None:
    """Une coupure pendant le démarrage ne doit pas faire perdre de réglages."""
    content = json.dumps({"KP": 0.33, "INCONNU": 1, "BASE_SPEED": 0.7})
    tuning.SETTINGS_FILE.write_text(content, encoding="utf-8")
    tuning.load()
    assert robot_utils.KP == 0.33 and robot_utils.BASE_SPEED == 0.7
    assert tuning.SETTINGS_FILE.read_text(encoding="utf-8") == content


def test_echec_d_ecriture_ne_bloque_pas_le_reglage(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr(tuning, "SETTINGS_FILE", tmp_path / "absent" / "settings.json")
    tuning.update({"KP": 0.33})
    assert robot_utils.KP == 0.33


# --- Serveur ---------------------------------------------------------------------------------

@pytest.fixture
def port() -> Any:
    server = TelemetryServer(port=0)
    yield server._server.server_address[1]
    server.close()


def post(port: int, path: str, payload: object, content_type: str = "application/json") -> Tuple[int, dict]:
    request = urllib.request.Request(f"http://127.0.0.1:{port}{path}", json.dumps(payload).encode(),
                                     {"Content-Type": content_type})
    try:
        with urllib.request.urlopen(request) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def test_api_lecture_et_modification(port: int) -> None:
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/settings") as response:
        listed = json.loads(response.read())["settings"]
    assert {"KP", "BASE_SPEED", "APPROACH_DURATION", "TURN_DURATION"} <= {e["key"] for e in listed}

    status, body = post(port, "/settings", {"values": {"KP": 0.25}})
    assert status == 200 and robot_utils.KP == 0.25
    assert next(e for e in body["settings"] if e["key"] == "KP")["value"] == 0.25

    status, _ = post(port, "/settings/reset", {"keys": ["KP"]})
    assert status == 200 and robot_utils.KP == tuning._DEFAULTS["KP"]


@pytest.mark.parametrize("payload", [{"values": {"KP": 99}}, {"values": "KP"}, {}, [], {"values": {"X": 1}}])
def test_api_refuse_les_requetes_invalides(port: int, payload: object) -> None:
    status, body = post(port, "/settings", payload)
    assert status == 400 and body["error"]
    assert robot_utils.KP == tuning._DEFAULTS["KP"]


def test_api_reset_ignore_les_cles_inconnues(port: int) -> None:
    assert post(port, "/settings/reset", {"keys": ["INCONNU"]})[0] == 200
    assert post(port, "/settings/reset", {"keys": "KP"})[0] == 400
    assert post(port, "/settings/reset", {"keys": [["KP"]]})[0] == 400


def test_autre_site_ne_peut_pas_changer_les_reglages(port: int) -> None:
    assert post(port, "/settings", {"values": {"KP": 0.5}}, content_type="text/plain")[0] == 415
    assert robot_utils.KP == tuning._DEFAULTS["KP"]


# --- Lancement direct ------------------------------------------------------------------------

def test_camera_pi_importable_comme_script(tmp_path: Path) -> None:
    """`python vision/camera_pi.py` place vision/ sur le chemin, pas la racine du projet."""
    code = (
        "import runpy, sys; sys.path[0] = sys.argv[1]; "
        "ns = runpy.run_path(sys.argv[2], run_name='camera_pi_script'); "
        "assert ns['STREAM_PORT'] == 8000"
    )
    subprocess.run([sys.executable, "-c", code, str(ROOT / "vision"), str(ROOT / "vision" / "camera_pi.py")],
                   check=True, cwd=str(tmp_path))
