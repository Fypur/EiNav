"""Vérifications de base sans interface ni robot connecté."""

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation import path_distance, read_map, shortest_path  # noqa: E402
from ligne import line_length, position_at, read_line, sample_line, validate_line  # noqa: E402


class TestCarte(unittest.TestCase):
    def setUp(self):
        self.carte = read_map(ROOT / "carte.json")

    def test_grille_4x4(self):
        self.assertEqual(len(self.carte["nodes"]), 16)
        self.assertEqual(len(self.carte["edges"]), 24)
        self.assertEqual(self.carte["start"], "M")
        positions = {(n["x"], n["y"]) for n in self.carte["nodes"]}
        self.assertEqual(positions, {(x, y) for x in range(4) for y in range(4)})

    def test_chemin_diagonale_grille(self):
        chemin = shortest_path(self.carte, "M", "D")
        self.assertEqual(chemin[0], "M")
        self.assertEqual(chemin[-1], "D")
        self.assertEqual(path_distance(self.carte, chemin), 6)


class TestLigne(unittest.TestCase):
    def setUp(self):
        self.ligne = read_line(ROOT / "ligne_blanche.json")
        self.points = sample_line(self.ligne)

    def test_courbe_lisse_et_progression(self):
        self.assertTrue(self.ligne["smooth"])
        self.assertGreater(len(self.points), len(self.ligne["points"]))
        start = (self.ligne["points"][0]["x"], self.ligne["points"][0]["y"])
        end = (self.ligne["points"][-1]["x"], self.ligne["points"][-1]["y"])
        self.assertEqual(position_at(self.points, 0), start)
        self.assertEqual(position_at(self.points, 1), end)
        self.assertGreater(line_length(self.points), math.dist(start, end))
        for t in (0, 0.1, 0.25, 0.5, 0.75, 0.95, 1):
            x, y = position_at(self.points, t)
            self.assertTrue(math.isfinite(x) and math.isfinite(y))

    def test_ligne_polygonale(self):
        line = {"smooth": False, "points": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 2}]}
        self.assertEqual(sample_line(line), [(0, 0), (1, 0), (1, 2)])
        self.assertEqual(position_at(sample_line(line), .5), (1, .5))

    def test_points_identiques_refuses(self):
        with self.assertRaises(ValueError):
            validate_line({"smooth": True, "points": [{"x": 0, "y": 0}, {"x": 0, "y": 0}]})


if __name__ == '__main__':
    unittest.main()
