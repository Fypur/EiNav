"""Commande des moteurs, avec accélérations et décélérations exponentielles."""
from __future__ import annotations

import math
from typing import Optional

from connections.serialpi import Serial

# Écart à la consigne en dessous duquel la vitesse est considérée comme atteinte
# (une exponentielle n'atteint jamais sa consigne).
SPEED_EPSILON = 0.01


class Control:
    """Filtre du 1er ordre sur les vitesses : v += (consigne - v) * (1 - exp(-dt / tau)).

    Propriété utile : un filtre linéaire conserve l'intégrale de sa consigne. Si la consigne
    de rotation dure T puis retombe à 0, l'angle parcouru une fois le robot arrêté est le même
    que sans rampe. `turn_duration` garde donc son sens, à condition d'attendre l'arrêt complet.

    La correction de suivi de ligne n'est pas filtrée : un retard dans la boucle de
    direction ferait osciller le robot autour de la ligne.
    """

    def __init__(self, serial: Serial, time_constant: float = 0.15) -> None:
        self.serial = serial
        self.kp = 0.15
        self.turn_duration = 1.5  # Duration of a 90-degree turn in seconds
        self.turn_speed = 0.8
        self.base_speed = 0.4
        self.forward_speed = 0.1
        self.time_constant = time_constant

        self._linear = 0.0
        self._angular = 0.0  # > 0 : rotation à droite
        self._last_time: Optional[float] = None

    @property
    def is_stopped(self) -> bool:
        return self._linear == 0.0 and self._angular == 0.0

    def _ramp(self, current: float, target: float, alpha: float) -> float:
        value = current + (target - current) * alpha
        return target if abs(target - value) < SPEED_EPSILON else value

    def _drive(self, now: float, linear: float, angular: float, correction: float = 0.0) -> None:
        dt = 0.0 if self._last_time is None else max(0.0, now - self._last_time)
        self._last_time = now
        alpha = 1.0 - math.exp(-dt / self.time_constant)
        self._linear = self._ramp(self._linear, linear, alpha)
        self._angular = self._ramp(self._angular, angular, alpha)

        left = self._linear + self._angular + correction
        right = self._linear - self._angular - correction
        self.serial.send_wheel_speeds(left, right, right, left)

    def stop(self, now: float) -> bool:
        """Décélère jusqu'à l'arrêt. Renvoie True quand le robot est arrêté."""
        self._drive(now, 0.0, 0.0)
        return self.is_stopped

    def halt(self) -> None:
        """Coupe les moteurs immédiatement, sans rampe (fin de programme, erreur)."""
        self._linear = 0.0
        self._angular = 0.0
        self.serial.send_wheel_speeds(0.0, 0.0, 0.0, 0.0)

    def forward(self, now: float) -> None:
        """Avance en ligne droite à vitesse réduite."""
        self._drive(now, self.forward_speed, 0.0)

    def aim(self, now: float, error: float) -> None:
        """Suit la ligne : `error` vaut -1 (ligne à gauche) à 1 (ligne à droite)."""
        self._drive(now, self.base_speed, 0.0, correction=error * self.kp)

    def turn(self, now: float, left: bool, start_time: float, quarter_turns: int = 1) -> bool:
        """Tourne sur place. Renvoie True quand la rotation est finie et le robot arrêté."""
        turning = now - start_time < self.turn_duration * quarter_turns
        speed = self.turn_speed if turning else 0.0
        self._drive(now, 0.0, -speed if left else speed)
        return not turning and self.is_stopped

    def approach_duration(self, duration_at_forward_speed: float) -> float:
        """Durée d'avance vers le centre d'une intersection, en tenant compte des rampes.

        En passant de v0 à la vitesse d'avance v puis à l'arrêt, le robot parcourt v * T + v0 * tau.
        On choisit T pour couvrir la distance qu'il aurait faite à v, sans rampe.
        """
        ramp_distance = self._linear * self.time_constant
        return max(0.0, duration_at_forward_speed - ramp_distance / self.forward_speed)
