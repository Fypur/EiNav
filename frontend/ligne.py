"""Géométrie de la ligne blanche (simulation UI, sans capteurs/moteurs)."""

from __future__ import annotations

import json
import math
from pathlib import Path


def validate_line(data: dict) -> dict:
    """Valide une ligne représentée par des points de contrôle cartésiens ordonnés."""
    if not isinstance(data, dict) or not isinstance(data.get("points"), list):
        raise ValueError("Le champ 'points' doit être une liste de coordonnées.")
    raw_points = data["points"]
    if not (2 <= len(raw_points) <= 200):
        raise ValueError("La ligne nécessite entre 2 et 200 points de contrôle.")
    points = []
    for index, raw in enumerate(raw_points, start=1):
        if not isinstance(raw, dict):
            raise ValueError(f"Point {index} : format invalide.")
        try:
            x, y = float(raw["x"]), float(raw["y"])
        except (KeyError, ValueError, TypeError):
            raise ValueError(f"Point {index} : coordonnées invalides.") from None
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"Point {index} : coordonnées non finies.")
        if points and math.hypot(x - points[-1]["x"], y - points[-1]["y"]) < 1e-9:
            raise ValueError(f"Point {index} : identique au point précédent.")
        points.append({"x": x, "y": y})
    smooth = data.get("smooth", True)
    if not isinstance(smooth, bool):
        raise ValueError("Le champ 'smooth' doit être un booléen.")
    return {"smooth": smooth, "points": points}


def read_line(filename: Path) -> dict:
    return validate_line(json.loads(filename.read_text(encoding="utf-8")))


def save_line(filename: Path, data: dict) -> dict:
    normalized = validate_line(data)
    filename.write_text(json.dumps(normalized, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return normalized


def sample_line(data: dict, samples_per_segment: int = 24) -> list[tuple[float, float]]:
    """Échantillonne une ligne droite par segment ou une courbe de Catmull-Rom.

    Les points de contrôle sont des points de passage. Les points intermédiaires
    définissent une courbe lisse (non limitée aux arêtes du maillage).
    """
    coords = [(p["x"], p["y"]) for p in data["points"]]
    if not data["smooth"] or len(coords) == 2:
        return coords
    if samples_per_segment < 1:
        raise ValueError("samples_per_segment doit être >= 1.")

    sampled: list[tuple[float, float]] = []
    for i in range(len(coords) - 1):
        p0 = coords[max(0, i - 1)]
        p1 = coords[i]
        p2 = coords[i + 1]
        p3 = coords[min(i + 2, len(coords) - 1)]
        for step in range(samples_per_segment):
            t = step / samples_per_segment
            t2, t3 = t * t, t * t * t
            xy = []
            for axis in (0, 1):
                value = 0.5 * (
                    2 * p1[axis]
                    + (-p0[axis] + p2[axis]) * t
                    + (2 * p0[axis] - 5 * p1[axis] + 4 * p2[axis] - p3[axis]) * t2
                    + (-p0[axis] + 3 * p1[axis] - 3 * p2[axis] + p3[axis]) * t3
                )
                xy.append(value)
            sampled.append((xy[0], xy[1]))
    sampled.append(coords[-1])
    return sampled


def line_length(samples: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(samples, samples[1:]))


def position_at(samples: list[tuple[float, float]], progress: float) -> tuple[float, float]:
    """Position simulée à une fraction [0, 1] de la longueur réelle de la ligne."""
    if not samples:
        raise ValueError("La ligne échantillonnée est vide.")
    if len(samples) == 1 or progress <= 0:
        return samples[0]
    if progress >= 1:
        return samples[-1]
    target = progress * line_length(samples)
    distance = 0.0
    for a, b in zip(samples, samples[1:]):
        segment_length = math.dist(a, b)
        if distance + segment_length >= target and segment_length > 0:
            frac = (target - distance) / segment_length
            return (a[0] + frac * (b[0] - a[0]), a[1] + frac * (b[1] - a[1]))
        distance += segment_length
    return samples[-1]
