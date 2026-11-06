"""Tests de la zone d'intérêt réglable (le rectangle bleu) : haut, bas, gauche et droite."""
from __future__ import annotations

from typing import List, Sequence, Tuple

import cv2
import numpy as np
import pytest

from robot import tuning
from robot.control import Control
from robot.state_machine import LineLossWatch, _steer
from vision import utils as vision_utils
from vision.image_processor import ImageProcessor, LineDetection

ROI_COLOR = np.array(vision_utils.ROI_COLOR)
FULL_LINE = [(70, 0), (90, 0), (90, 127), (70, 127)]


def _frame(*polygons: Sequence[Sequence[Tuple[int, int]]]) -> np.ndarray:
    """Polygones blancs sur un sol sombre, coordonnées dans l'image 160 x 128."""
    frame = np.full((128, 160, 3), 40, np.uint8)
    for polygon in polygons:
        cv2.fillPoly(frame, [np.array(polygon, np.int32)], (220, 220, 220))
    return frame


def _process(frame: np.ndarray) -> LineDetection:
    return ImageProcessor().process_frame(frame)


class FakeSerial:
    def __init__(self) -> None:
        self.speeds: List[Tuple[float, float, float, float]] = []

    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        self.speeds.append((fl, fr, rr, rl))


def test_valeurs_par_defaut_toute_la_largeur_jusqu_en_bas() -> None:
    detection = _process(_frame(FULL_LINE))
    assert detection.mask.shape == (128 - 76, 160)
    assert detection.roi_top == 76 and detection.roi_left == 0


def test_la_barre_du_bas_ignore_ce_qui_est_dessous() -> None:
    bottom_patch = [(70, 110), (90, 110), (90, 127), (70, 127)]
    assert _process(_frame(bottom_patch)).cx is not None
    tuning.update({"ROI_BOTTOM_RATIO": 0.8})  # la zone finit à la ligne 102
    detection = _process(_frame(bottom_patch))
    assert detection.mask.shape[0] == 102 - 76
    assert detection.cx is None


@pytest.mark.parametrize("key,ratio,patch", [
    ("ROI_LEFT_RATIO", 0.25, [(5, 80), (30, 80), (30, 127), (5, 127)]),
    ("ROI_RIGHT_RATIO", 0.75, [(130, 80), (155, 80), (155, 127), (130, 127)]),
])
def test_les_barres_laterales_ignorent_ce_qui_est_a_cote(key: str, ratio: float, patch: List[Tuple[int, int]]) -> None:
    assert _process(_frame(patch)).cx is not None
    tuning.update({key: ratio})
    assert _process(_frame(patch)).cx is None


def test_cx_reste_en_coordonnees_de_l_image() -> None:
    """Même ligne, même cx, quelle que soit la position de la barre de gauche."""
    before = _process(_frame(FULL_LINE)).cx
    tuning.update({"ROI_LEFT_RATIO": 0.25})
    detection = _process(_frame(FULL_LINE))
    assert detection.cx == before
    assert detection.roi_left == 40 and detection.mask.shape[1] == 120


def test_le_suivi_vise_toujours_le_milieu_de_l_image() -> None:
    """Une zone décalée ne doit pas faire dériver le robot : ligne au milieu, pas de correction."""
    tuning.update({"ROI_LEFT_RATIO": 0.4})  # la zone commence à 64 pixels, son milieu est à 112
    detection = _process(_frame([(72, 0), (88, 0), (88, 127), (72, 127)]))  # ligne au milieu de l'image
    assert detection.image_w == 160
    serial = FakeSerial()
    _steer(Control(serial), detection, 0.0)  # type: ignore[arg-type]
    left, right, _, _ = serial.speeds[-1]
    assert abs(left - right) < 0.01


def test_le_cote_de_la_derniere_ligne_vue_se_mesure_sur_l_image() -> None:
    tuning.update({"ROI_RIGHT_RATIO": 0.75})  # la zone finit à 120 pixels, son milieu est à 60
    watch = LineLossWatch()
    watch.last_turn_left = False
    watch.update(_process(_frame([(62, 0), (78, 0), (78, 127), (62, 127)])))  # à droite de 60, à gauche de 80
    assert watch.last_turn_left is True


def test_le_rectangle_affiche_suit_les_quatre_barres() -> None:
    tuning.update({"ROI_TOP_RATIO": 0.5, "ROI_BOTTOM_RATIO": 0.75, "ROI_LEFT_RATIO": 0.25, "ROI_RIGHT_RATIO": 0.75})
    processor = ImageProcessor()
    frame = _frame()
    image = processor.annotate(frame, processor.process_frame(frame))
    is_roi = np.all(image == ROI_COLOR, axis=2)
    rows, cols = np.nonzero(is_roi)
    assert (rows.min(), rows.max(), cols.min(), cols.max()) == (64, 95, 40, 119)


def test_la_ligne_detectee_est_dessinee_au_bon_endroit() -> None:
    tuning.update({"ROI_LEFT_RATIO": 0.25, "ROI_BOTTOM_RATIO": 0.8})
    processor = ImageProcessor()
    frame = _frame(FULL_LINE)
    image = processor.annotate(frame, processor.process_frame(frame))
    is_line = np.all(image == np.array(vision_utils.LINE_COLOR), axis=2)
    rows, cols = np.nonzero(is_line)
    assert rows.min() >= 76 and rows.max() < 102
    assert cols.min() >= 66 and cols.max() <= 94
    target_rows, _ = np.nonzero(np.all(image == np.array(vision_utils.TARGET_COLOR), axis=2))
    assert (target_rows.min(), target_rows.max()) == (76, 101)  # le trait rouge reste dans la zone


@pytest.mark.parametrize("values", [
    {"ROI_TOP_RATIO": 0.9, "ROI_BOTTOM_RATIO": 0.2},
    {"ROI_LEFT_RATIO": 0.9, "ROI_RIGHT_RATIO": 0.1},
    {"ROI_TOP_RATIO": 0.9, "ROI_BOTTOM_RATIO": 0.2, "ROI_LEFT_RATIO": 0.9, "ROI_RIGHT_RATIO": 0.1},
    {"ROI_TOP_RATIO": 0.5, "ROI_BOTTOM_RATIO": 0.5},
])
def test_des_barres_croisees_ne_font_pas_planter(values: dict) -> None:
    tuning.update(values)
    processor = ImageProcessor()
    frame = _frame(FULL_LINE)
    detection = processor.process_frame(frame)
    assert min(detection.mask.shape) >= vision_utils.ROI_MIN_PIXELS
    processor.annotate(frame, detection)


def test_zone_etroite_une_simple_ligne_n_est_pas_un_croisement() -> None:
    """La ligne touche le haut, le bas et le bord de la zone, mais reste moins large qu'un croisement."""
    tuning.update({"ROI_LEFT_RATIO": 0.3, "ROI_RIGHT_RATIO": 0.7})  # zone de 64 pixels
    line = [(48, 0), (88, 0), (88, 127), (48, 127)]  # 40 pixels de large, coupée par la barre de gauche
    detection = _process(_frame(line))
    assert detection.cx is not None
    assert not detection.is_intersection


def test_croisement_toujours_vu_dans_une_zone_reduite() -> None:
    tuning.update({"ROI_LEFT_RATIO": 0.1, "ROI_RIGHT_RATIO": 0.9})
    branch = [(0, 95), (160, 95), (160, 110), (0, 110)]
    assert _process(_frame(FULL_LINE, branch)).is_intersection


@pytest.mark.parametrize("key", ["ROI_BOTTOM_RATIO", "ROI_LEFT_RATIO", "ROI_RIGHT_RATIO"])
def test_reglable_depuis_l_interface(key: str) -> None:
    described = {s["key"]: s for s in tuning.describe()}
    assert key in described and described[key]["group"] == "Vision"
    tuning.update({key: 0.5})
    assert getattr(vision_utils, key) == 0.5
    tuning.reset([key])
    assert getattr(vision_utils, key) == described[key]["default"]
    with pytest.raises(ValueError):
        tuning.update({key: 1.5})
