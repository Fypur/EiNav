"""Réglages qu'on peut changer depuis l'interface web. Ils s'appliquent direct et restent après un redémarrage.

Les valeurs par défaut sont dans les utils.py. settings.json (pas versionné) garde seulement
les valeurs qui ont changé, donc modifier un utils.py marche toujours pour les autres réglages.

Pour ajouter un réglage, ajouter une ligne dans SETTINGS avec des bornes qui ne font pas
planter le robot. Dans le code, il faut lire la constante avec utils.NOM.
"""
from __future__ import annotations

import json
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any, Dict, Iterable, List, Mapping, Optional

from connections import utils as connections_utils
from robot import utils as robot_utils
from vision import utils as vision_utils

SETTINGS_FILE = Path(__file__).resolve().parents[1] / "settings.json"


@dataclass(frozen=True)
class Setting:
    key: str  # nom de la constante dans le module
    module: ModuleType
    group: str
    label: str
    help: str
    minimum: float
    maximum: float
    step: float
    unit: str = ""
    integer: bool = False


SETTINGS = (
    Setting("BASE_SPEED", robot_utils, "Déplacement", "Vitesse sur la ligne",
            "Vitesse en suivi de ligne, en fraction de la vitesse maximale.", 0.05, 1.0, 0.01),
    Setting("FORWARD_SPEED", robot_utils, "Déplacement", "Vitesse d'approche",
            "Vitesse à l'approche d'un croisement, ou quand la ligne est perdue.", 0.02, 1.0, 0.01),
    Setting("RAMP_TIME_CONSTANT", robot_utils, "Déplacement", "Douceur des rampes",
            "Plus la valeur est grande, plus le robot accélère et freine doucement.", 0.01, 1.0, 0.01, "s"),
    Setting("KP", robot_utils, "Suivi de ligne", "Force de correction (kp)",
            "Vitesse ajoutée à une roue et retirée à l'autre quand la ligne est au bord de l'image.",
            0.0, 1.0, 0.01),
    Setting("TIME_STEP_MS", connections_utils, "Suivi de ligne", "Période de la boucle",
            "Temps d'attente entre deux images traitées, sur le robot réel. Plus court : "
            "réagit plus vite, mais charge davantage le Pi.", 10, 200, 1, "ms", integer=True),
    Setting("LINE_LOST_TIMEOUT", robot_utils, "Suivi de ligne", "Délai de recherche de ligne",
            "Temps maximal sans voir la ligne avant l'arrêt complet du robot.", 0.1, 10.0, 0.1, "s"),
    Setting("APPROACH_DURATION", robot_utils, "Croisements", "Durée d'approche",
            "Temps pour atteindre le centre du croisement après l'avoir détecté. "
            "Le robot s'arrête trop tôt : augmenter.", 0.0, 3.0, 0.05, "s"),
    Setting("TURN_SPEED", robot_utils, "Croisements", "Vitesse de rotation",
            "Vitesse des roues pendant un virage sur place.", 0.1, 1.0, 0.01),
    Setting("TURN_90_DURATION", robot_utils, "Croisements", "Durée d'une rotation de 90°",
            "Virages à gauche et à droite. Le robot tourne trop peu : augmenter.",
            0.1, 5.0, 0.05, "s"),
    Setting("TURN_180_DURATION", robot_utils, "Croisements", "Durée d'un demi-tour",
            "Rotation de 180°, à destination notamment. Le robot tourne trop peu : augmenter.",
            0.1, 10.0, 0.05, "s"),
    Setting("BINARY_THRESHOLD", vision_utils, "Vision", "Seuil de blanc",
            "Niveau de gris à partir duquel un pixel fait partie de la ligne. "
            "Baisser si la ligne n'est pas vue, monter si le sol passe en vert.",
            0, 255, 1, integer=True),
    Setting("ROI_TOP_RATIO", vision_utils, "Vision", "Haut de la zone analysée",
            "Fraction de la hauteur de l'image où commence la zone analysée. "
            "Plus petit : le robot regarde plus loin.", 0.1, 0.9, 0.05),
    Setting("MIN_LINE_AREA", vision_utils, "Vision", "Surface minimale de ligne",
            "En dessous de cette surface, une tache blanche est ignorée.", 0, 2000, 10, "px", integer=True),
    Setting("INTERSECTION_MIN_WIDTH_RATIO", vision_utils, "Vision", "Largeur d'un croisement",
            "Largeur minimale de la bande blanche d'un croisement, en fraction de l'image. Tourne au milieu "
            "d'une ligne : augmenter. Passe des croisements sans les voir : baisser.", 0.1, 1.0, 0.05),
)

_BY_KEY: Dict[str, Setting] = {setting.key: setting for setting in SETTINGS}
_DEFAULTS: Dict[str, float] = {setting.key: getattr(setting.module, setting.key) for setting in SETTINGS}
_lock = threading.Lock()


def _validated(setting: Setting, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{setting.label} : valeur numérique attendue.")
    # Écrit comme ça, le test rejette aussi NaN, l'infini et les entiers énormes que json.loads accepte
    if not setting.minimum <= value <= setting.maximum:
        raise ValueError(f"{setting.label} : doit être entre {setting.minimum:g} et {setting.maximum:g}.")
    if setting.integer:
        if value != int(value):
            raise ValueError(f"{setting.label} : valeur entière attendue.")
        return int(value)
    return float(value)


def describe() -> List[Dict[str, Any]]:
    """Liste des réglages avec leurs valeurs, pour l'interface."""
    return [
        {"key": s.key, "group": s.group, "label": s.label, "help": s.help, "unit": s.unit,
         "min": s.minimum, "max": s.maximum, "step": s.step,
         "value": getattr(s.module, s.key), "default": _DEFAULTS[s.key]}
        for s in SETTINGS
    ]


def _save() -> None:
    """On écrit dans un fichier temporaire puis on remplace, pour éviter un fichier à moitié écrit si le courant coupe."""
    overrides = {s.key: getattr(s.module, s.key) for s in SETTINGS
                 if getattr(s.module, s.key) != _DEFAULTS[s.key]}
    temporary = SETTINGS_FILE.with_suffix(".tmp")
    try:
        temporary.write_text(json.dumps(overrides, indent=2) + "\n", encoding="utf-8")
        os.replace(str(temporary), str(SETTINGS_FILE))
    except OSError as exc:
        print(f"[réglages] impossible d'enregistrer {SETTINGS_FILE} : {exc}")


def update(values: Mapping[str, object]) -> None:
    """Applique les réglages. Si l'un d'eux est invalide, ValueError et on n'applique rien."""
    checked = {}
    for key, value in values.items():
        if key not in _BY_KEY:
            raise ValueError(f"Réglage inconnu : {key}.")
        checked[key] = _validated(_BY_KEY[key], value)
    with _lock:
        for key, value in checked.items():
            setattr(_BY_KEY[key].module, key, value)
        _save()


def reset(keys: Optional[Iterable[str]] = None) -> None:
    """Revient aux valeurs par défaut des réglages donnés, ou de tous."""
    update({key: _DEFAULTS[key] for key in (_DEFAULTS if keys is None else keys) if key in _DEFAULTS})


def load() -> None:
    """Recharge les réglages enregistrés. On ignore les valeurs invalides pour que le robot démarre quand même."""
    try:
        saved = json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return
    except (OSError, ValueError) as exc:
        print(f"[réglages] {SETTINGS_FILE} illisible, valeurs par défaut utilisées : {exc}")
        return
    if not isinstance(saved, dict):
        print(f"[réglages] {SETTINGS_FILE} illisible, valeurs par défaut utilisées")
        return
    for key, value in saved.items():
        if key not in _BY_KEY:
            print(f"[réglages] ignoré : réglage inconnu {key}.")
            continue
        try:
            # Pas de _save() ici, sinon une coupure au démarrage peut faire perdre des réglages
            setattr(_BY_KEY[key].module, key, _validated(_BY_KEY[key], value))
        except ValueError as exc:
            print(f"[réglages] ignoré : {exc}")
    if saved:
        print("[réglages] valeurs modifiées depuis l'interface :",
              ", ".join(f"{s.key}={getattr(s.module, s.key)}" for s in SETTINGS
                        if getattr(s.module, s.key) != _DEFAULTS[s.key]))
