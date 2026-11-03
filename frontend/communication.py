"""Passerelle de commandes Streamlit -> Arduino (USB série).

Par défaut : simulation, aucun Arduino nécessaire.
Le protocole série utilise un objet JSON UTF-8 par ligne (\n).

Exemples envoyés :
    {"command":"goto","x":2,"y":1}
    {"command":"follow_line"}
    {"command":"return_home"}
    {"command":"stop"}

Exemple de réponse attendue de l'Arduino :
    {"status":"ARRIVED","x":2,"y":1}
    {"status":"MOVING","x":1,"y":1}
    {"status":"ERROR","message":"ligne perdue"}

Ce module ne contrôle PAS les moteurs directement. L'Arduino devra
implémenter la réception et l'exécution de ces commandes plus tard.
"""

from __future__ import annotations

import json
import time
from typing import Any


class CommunicationRobot:
    """API unique pour les boutons Streamlit et la future liaison Arduino."""

    def __init__(
        self,
        mode: str = "simulation",
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
    ) -> None:
        if mode not in {"simulation", "serie"}:
            raise ValueError("mode doit valoir 'simulation' ou 'serie'")

        self.mode = mode
        self._serial = None
        self._tampon = bytearray()
        self._fin_simulation: float | None = None
        self._destination_simulee: tuple[int, int] | None = None
        self._etat: dict[str, Any] = {
            "status": "IDLE",
            "x": 0,
            "y": 0,
            "message": "Prêt",
        }
        self.derniere_commande: dict[str, Any] | None = None

        if mode == "serie":
            try:
                import serial  # pyserial : nécessaire seulement en mode série
                self._serial = serial.Serial(
                    port=port, baudrate=baudrate,
                    timeout=0, write_timeout=1,
                )
            except Exception as exc:
                raise ConnectionError(
                    f"Impossible d'ouvrir le port Arduino {port}: {exc}"
                ) from exc

    def _envoyer(self, commande: dict[str, Any], destination=None) -> None:
        """Envoie une ligne JSON (ou simule la commande)."""
        self.derniere_commande = commande.copy()
        if self.mode == "simulation":
            self._etat.update(status="MOVING", message="Déplacement simulé")
            self._destination_simulee = destination
            self._fin_simulation = time.monotonic() + 2.0
            return

        try:
            ligne = (json.dumps(commande, separators=(",", ":")) + "\n")
            self._serial.write(ligne.encode("utf-8"))
            self._etat.update(status="SENT", message="Commande envoyée ; attente Arduino")
        except Exception as exc:
            self._etat.update(status="ERROR", message=str(exc))
            raise ConnectionError(f"Erreur de communication série : {exc}") from exc

    def partir_destination(self, x: int, y: int) -> None:
        """Demande d'aller au nœud cartésien (x, y) du maillage 4x4."""
        if not isinstance(x, int) or not isinstance(y, int) or not (0 <= x <= 3 and 0 <= y <= 3):
            raise ValueError("X et Y doivent être des entiers entre 0 et 3")
        self._envoyer({"command": "goto", "x": x, "y": y}, (x, y))

    def demarrer_suivi_ligne(self) -> None:
        """Active le suivi physique de ligne, droite ou courbe."""
        self._envoyer({"command": "follow_line"}, None)

    def retour_depart(self) -> None:
        """Demande le retour au point (0, 0)."""
        self._envoyer({"command": "return_home"}, (0, 0))

    def stop(self) -> None:
        """Commande d'arrêt (ne remplace pas une sécurité matérielle)."""
        self.derniere_commande = {"command": "stop"}
        if self.mode == "serie":
            try:
                self._serial.write(b'{"command":"stop"}\n')
            except Exception as exc:
                self._etat.update(status="ERROR", message=str(exc))
                raise ConnectionError(f"Erreur d'envoi STOP : {exc}") from exc
        self._fin_simulation = None
        if self.mode == "simulation":
            self._etat.update(status="IDLE", message="Arrêt simulé")
        else:
            self._etat.update(status="SENT", message="STOP envoyé ; attendre confirmation Arduino")

    def lire_etat(self) -> dict[str, Any]:
        """Renvoie le dernier état; ne bloque pas la page Streamlit."""
        if self.mode == "simulation":
            if self._fin_simulation is not None and time.monotonic() >= self._fin_simulation:
                if self._destination_simulee is not None:
                    self._etat["x"], self._etat["y"] = self._destination_simulee
                self._etat.update(status="ARRIVED", message="Arrivée simulée")
                self._fin_simulation = None
            return self._etat.copy()

        try:
            n = self._serial.in_waiting
            if n:
                self._tampon.extend(self._serial.read(n))
                if len(self._tampon) > 8192:
                    self._tampon.clear()
                    raise ValueError("Message série trop volumineux")
                while b"\n" in self._tampon:
                    ligne, _, reste = self._tampon.partition(b"\n")
                    self._tampon = bytearray(reste)
                    try:
                        reponse = json.loads(ligne.decode("utf-8"))
                    except (ValueError, UnicodeDecodeError):
                        continue
                    if not isinstance(reponse, dict):
                        continue
                    statut = reponse.get("status")
                    if statut in {"IDLE", "MOVING", "ARRIVED", "ERROR"}:
                        self._etat["status"] = statut
                    for champ in ("x", "y", "message"):
                        if champ in reponse:
                            self._etat[champ] = reponse[champ]
        except Exception as exc:
            self._etat.update(status="ERROR", message=f"Erreur série : {exc}")
        return self._etat.copy()

    def fermer(self) -> None:
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
