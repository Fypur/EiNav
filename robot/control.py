"""Commande des moteurs, avec accélérations et décélérations exponentielles."""
from __future__ import annotations

import math
from typing import Optional

from connections.serialpi import Serial
from robot import utils


class Control:
    """Filtre du 1er ordre sur les vitesses : v += (consigne - v) * (1 - exp(-dt / tau)).

    Propriété utile : un filtre linéaire conserve l'intégrale de sa consigne. Si la consigne
    de rotation dure T puis retombe à 0, l'angle parcouru une fois le robot arrêté est le même
    que sans rampe. Les durées de rotation gardent donc leur sens, à condition d'attendre l'arrêt complet.

    La correction de suivi de ligne n'est pas filtrée : un retard dans la boucle de
    direction ferait osciller le robot autour de la ligne.

    Les réglages sont relus dans `utils` à chaque commande : un changement fait depuis
    l'interface s'applique pendant la mission.
    """

    def __init__(self, serial: Serial) -> None:
        self.serial = serial

        self._linear = 0.0
        self._angular = 0.0  # > 0 : rotation à droite
        self._last_time: Optional[float] = None
        self._speed_cap = math.inf
        # Temps passé sans être bloqué par un obstacle : un robot immobile n'avance pas vers
        # le centre d'un croisement, et ne perd pas la ligne.
        self.drive_time = 0.0
        # Distance parcourue depuis le dernier nœud, en vitesse x secondes.
        self.travel = 0.0

    @property
    def is_stopped(self) -> bool:
        return self._linear == 0.0 and self._angular == 0.0

    def _ramp(self, current: float, target: float, alpha: float) -> float:
        value = current + (target - current) * alpha
        return target if abs(target - value) < utils.SPEED_EPSILON else value

    def set_speed_cap(self, cap: float) -> None:
        """Limite la vitesse d'avance (obstacle). Les rotations sur place ne sont pas concernées."""
        self._speed_cap = cap

    def _drive(self, now: float, linear: float, angular: float, correction: float = 0.0) -> None:
        dt = 0.0 if self._last_time is None else max(0.0, now - self._last_time)
        self._last_time = now
        if self._speed_cap > 0.0:
            self.drive_time += dt
        alpha = 1.0 - math.exp(-dt / utils.RAMP_TIME_CONSTANT)
        # Le plafond s'applique aussi après le filtre : freiner devant un obstacle se fait sans retard.
        self._linear = min(self._ramp(self._linear, min(linear, self._speed_cap), alpha), self._speed_cap)
        self._angular = self._ramp(self._angular, angular, alpha)
        self.travel += self._linear * dt
        # La correction suit la vitesse : la trajectoire reste la même, et à l'arrêt le robot ne pivote pas.
        if linear > self._speed_cap:
            correction *= self._speed_cap / linear

        limit = utils.MAX_WHEEL_SPEED
        left = max(-limit, min(limit, self._linear + self._angular + correction))
        right = max(-limit, min(limit, self._linear - self._angular - correction))
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
        self._drive(now, utils.FORWARD_SPEED, 0.0)

    def aim(self, now: float, error: float) -> None:
        """Suit la ligne : `error` vaut -1 (ligne à gauche) à 1 (ligne à droite)."""
        self._drive(now, utils.BASE_SPEED, 0.0, correction=error * utils.KP)

    def turn(self, now: float, left: bool, start_time: float, half_turn: bool = False) -> bool:
        """Tourne sur place, de 90° ou 180°. Renvoie True quand la rotation est finie et le robot arrêté."""
        duration = utils.TURN_180_DURATION if half_turn else utils.TURN_90_DURATION
        turning = now - start_time < duration
        speed = utils.TURN_SPEED if turning else 0.0
        self._drive(now, 0.0, -speed if left else speed)
        return not turning and self.is_stopped

    def approach_duration(self, duration_at_forward_speed: float) -> float:
        """Durée d'avance vers le centre d'une intersection, en tenant compte des rampes.

        En passant de v0 à la vitesse d'avance v puis à l'arrêt, le robot parcourt v * T + v0 * tau.
        On choisit T pour couvrir la distance qu'il aurait faite à v, sans rampe.
        """
        ramp_distance = self._linear * utils.RAMP_TIME_CONSTANT
        return max(0.0, duration_at_forward_speed - ramp_distance / utils.FORWARD_SPEED)
