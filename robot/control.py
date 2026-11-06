"""Commande des moteurs avec des rampes d'accélération et de freinage."""
from __future__ import annotations

import math
from typing import Optional

from connections.serialpi import Serial
from robot import utils


class Control:
    """Filtre du 1er ordre sur les vitesses : v += (consigne - v) * (1 - exp(-dt / tau)).

    Le filtre conserve l'intégrale de la consigne, donc on tourne du même angle avec ou sans
    rampe. Les durées de rotation restent bonnes, mais il faut attendre l'arrêt complet.

    On ne filtre pas la correction du suivi de ligne, le retard ferait osciller le robot.

    Les réglages sont relus dans utils à chaque commande pour pouvoir les changer en pleine mission.
    """

    def __init__(self, serial: Serial) -> None:
        self.serial = serial

        self._linear = 0.0
        self._angular = 0.0  # > 0 : rotation à droite
        self._last_time: Optional[float] = None
        self._speed_cap = math.inf
        # Temps de conduite sans compter les arrêts devant un obstacle (à l'arrêt on n'avance
        # pas vers le croisement et on ne perd pas la ligne)
        self.drive_time = 0.0
        # Distance depuis le dernier noeud, en vitesse x secondes
        self.travel = 0.0

    @property
    def is_stopped(self) -> bool:
        return self._linear == 0.0 and self._angular == 0.0

    def _ramp(self, current: float, target: float, alpha: float) -> float:
        value = current + (target - current) * alpha
        return target if abs(target - value) < utils.SPEED_EPSILON else value

    def set_speed_cap(self, cap: float) -> None:
        """Limite la vitesse d'avance quand il y a un obstacle (pas les rotations sur place)."""
        self._speed_cap = cap

    def _drive(self, now: float, linear: float, angular: float, correction: float = 0.0) -> None:
        dt = 0.0 if self._last_time is None else max(0.0, now - self._last_time)
        self._last_time = now
        if self._speed_cap > 0.0:
            self.drive_time += dt
        alpha = 1.0 - math.exp(-dt / utils.RAMP_TIME_CONSTANT)
        # On plafonne aussi après le filtre pour freiner sans retard devant un obstacle
        self._linear = min(self._ramp(self._linear, min(linear, self._speed_cap), alpha), self._speed_cap)
        self._angular = self._ramp(self._angular, angular, alpha)
        self.travel += self._linear * dt
        # On réduit la correction avec la vitesse, sinon le robot pivote quand il est à l'arrêt
        if linear > self._speed_cap:
            correction *= self._speed_cap / linear

        limit = utils.MAX_WHEEL_SPEED
        left = max(-limit, min(limit, self._linear + self._angular + correction))
        right = max(-limit, min(limit, self._linear - self._angular - correction))
        self.serial.send_wheel_speeds(left, right, right, left)

    def stop(self, now: float) -> bool:
        """Freine jusqu'à l'arrêt. Renvoie True quand le robot est arrêté."""
        self._drive(now, 0.0, 0.0)
        return self.is_stopped

    def halt(self) -> None:
        """Coupe les moteurs direct, sans rampe (fin du programme ou erreur)."""
        self._linear = 0.0
        self._angular = 0.0
        self.serial.send_wheel_speeds(0.0, 0.0, 0.0, 0.0)

    def forward(self, now: float) -> None:
        """Avance en ligne droite à vitesse réduite."""
        self._drive(now, utils.FORWARD_SPEED, 0.0)

    def aim(self, now: float, error: float) -> None:
        """Suivi de ligne. error va de -1 (ligne à gauche) à 1 (ligne à droite)."""
        self._drive(now, utils.BASE_SPEED, 0.0, correction=error * utils.KP)

    def spin(self, now: float, left: bool, speed: Optional[float] = None) -> None:
        """Pivote sur place pour retrouver la ligne (à TURN_SPEED si on ne donne pas de vitesse)."""
        speed = utils.TURN_SPEED if speed is None else speed
        self._drive(now, 0.0, -speed if left else speed)

    def turn(self, now: float, left: bool, start_time: float, half_turn: bool = False) -> bool:
        """Rotation sur place de 90° ou 180°. Renvoie True une fois la rotation finie et le robot arrêté."""
        duration = utils.TURN_180_DURATION if half_turn else utils.TURN_90_DURATION
        turning = now - start_time < duration
        speed = utils.TURN_SPEED if turning else 0.0
        self._drive(now, 0.0, -speed if left else speed)
        return not turning and self.is_stopped

    def approach_duration(self, duration_at_forward_speed: float) -> float:
        """Durée d'avance jusqu'au centre du croisement en tenant compte des rampes.

        En passant de v0 à v puis à l'arrêt, le robot parcourt v * T + v0 * tau.
        On prend T pour faire la même distance que sans rampe.
        """
        ramp_distance = self._linear * utils.RAMP_TIME_CONSTANT
        return max(0.0, duration_at_forward_speed - ramp_distance / utils.FORWARD_SPEED)
