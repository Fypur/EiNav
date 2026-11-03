
import json
import time


# =========================================
# VALIDATION DES NOEUDS ET DES ARÊTES
# =========================================

def noeud_valide(value):
    return (
        isinstance(value, (list, tuple))
        and len(value) == 2
        and all(
            type(v) is int and 0 <= v <= 3
            for v in value
        )
    )


def arete_valide(a, b):
    return (
        noeud_valide(a)
        and noeud_valide(b)
        and abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
    )


# =========================================
# CLASSE DE COMMUNICATION
# =========================================

class CommunicationRobot:

    def __init__(
        self,
        mode="simulation",
        port="/dev/ttyACM0",
        baudrate=115200
    ):

        if mode not in ("simulation", "serie"):
            raise ValueError("Mode de communication invalide")

        self.mode = mode
        self.derniere_commande = None

        self._port = None
        self._buffer = bytearray()
        self._dernier_recu = None

        # État connu du robot
        self._etat = {
            "status": "IDLE",
            "node": [0, 0],
            "from": None,
            "to": None,
            "traversed": [],
            "message": "Prêt"
        }

        # Simulation
        self._sim_chemin = []
        self._sim_index = 0
        self._sim_t = time.monotonic()
        self._sim_s_par_arete = 1.2

        # Connexion réelle Arduino
        if mode == "serie":

            try:
                import serial

                self._port = serial.Serial(
                    port,
                    baudrate=baudrate,
                    timeout=0,
                    write_timeout=1
                )

            except Exception as exc:
                raise ConnectionError(
                    f"Impossible de connecter Arduino : {exc}"
                ) from exc

    # =====================================
    # ÉTAT PUBLIC DU ROBOT
    # =====================================

    def _etat_public(self):

        resultat = self._etat.copy()

        resultat["traversed"] = list(
            self._etat["traversed"]
        )

        if self._dernier_recu is None:
            resultat["age_s"] = None
        else:
            resultat["age_s"] = round(
                time.monotonic() - self._dernier_recu,
                2
            )

        return resultat

    # =====================================
    # CALCUL DE CHEMIN POUR SIMULATION
    # =====================================

    @staticmethod
    def _chemin(start, goal):

        chemin = [list(start)]

        x, y = start

        while x != goal[0]:

            x += (
                1 if goal[0] > x else -1
            )

            chemin.append([x, y])

        while y != goal[1]:

            y += (
                1 if goal[1] > y else -1
            )

            chemin.append([x, y])

        return chemin

    # =====================================
    # ENVOI DE COMMANDE
    # =====================================

    def _envoyer(self, commande):

        self.derniere_commande = commande.copy()

        if self.mode == "serie":

            try:

                payload = (
                    json.dumps(commande) + "\n"
                )

                self._port.write(
                    payload.encode("utf-8")
                )

                self._etat.update(
                    status="SENT",
                    message="Commande envoyée"
                )

                self._dernier_recu = None

            except Exception as exc:

                self._etat.update(
                    status="ERROR",
                    message=str(exc)
                )

                raise ConnectionError(
                    str(exc)
                ) from exc

    # =====================================
    # DÉMARRAGE SIMULATION
    # =====================================

    def _demarrer_simulation(self, chemin):

        self._sim_chemin = chemin

        self._sim_index = 0

        self._sim_t = time.monotonic()

        self._etat["traversed"] = []

        self._etat["node"] = list(chemin[0])

        if len(chemin) > 1:

            self._etat.update(
                status="MOVING",
                **{
                    "from": list(chemin[0]),
                    "to": list(chemin[1])
                }
            )

        else:

            self._etat.update(
                status="ARRIVED",
                **{
                    "from": None,
                    "to": None
                }
            )

    # =====================================
    # COMMANDE DESTINATION
    # =====================================

    def partir_destination(self, x, y):

        if not noeud_valide([x, y]):
            raise ValueError(
                "X et Y doivent être entre 0 et 3"
            )

        self._envoyer({
            "command": "goto",
            "x": x,
            "y": y
        })

        if self.mode == "simulation":

            chemin = self._chemin(
                self._etat["node"],
                (x, y)
            )

            self._demarrer_simulation(chemin)

    # =====================================
    # COMMANDE SUIVI DE LIGNE
    # =====================================

    def demarrer_suivi_ligne(self):

        self._envoyer({
            "command": "follow_line"
        })

        if self.mode == "simulation":

            self._sim_chemin = []

            self._etat.update(
                status="MOVING",
                **{
                    "from": None,
                    "to": None
                },
                message="Suivi de ligne en cours"
            )

    # =====================================
    # COMMANDE RETOUR AU DÉPART
    # =====================================

    def retour_depart(self):

        self._envoyer({
            "command": "return_home"
        })

        if self.mode == "simulation":

            chemin = self._chemin(
                self._etat["node"],
                (0, 0)
            )

            self._demarrer_simulation(chemin)

    # =====================================
    # COMMANDE STOP
    # =====================================

    def stop(self):

        self._envoyer({
            "command": "stop"
        })

        if self.mode == "simulation":

            self._sim_chemin = []

            self._etat.update(
                status="IDLE",
                **{
                    "from": None,
                    "to": None
                }
            )

    # =====================================
    # AVANCEMENT EN SIMULATION
    # =====================================

    def _simuler(self):

        if (
            self._etat["status"] != "MOVING"
            or len(self._sim_chemin) < 2
        ):
            return

        maintenant = time.monotonic()

        while (
            self._etat["status"] == "MOVING"
            and maintenant - self._sim_t
            >= self._sim_s_par_arete
        ):

            self._sim_t += self._sim_s_par_arete

            precedent = self._sim_chemin[
                self._sim_index
            ]

            self._sim_index += 1

            courant = self._sim_chemin[
                self._sim_index
            ]

            # Nouveau noeud confirmé
            self._etat["node"] = list(courant)

            # Marquer l'arête comme parcourue
            self._etat["traversed"].append([
                list(precedent),
                list(courant)
            ])

            # Vérifier si destination atteinte
            if (
                self._sim_index
                == len(self._sim_chemin) - 1
            ):

                self._etat.update(
                    status="ARRIVED",
                    **{
                        "from": None,
                        "to": None
                    }
                )

            else:

                suivant = self._sim_chemin[
                    self._sim_index + 1
                ]

                self._etat.update(
                    **{
                        "from": list(courant),
                        "to": list(suivant)
                    }
                )

    # =====================================
    # TRAITEMENT DES MESSAGES ARDUINO
    # =====================================

    def _integrer_trame(self, msg):

        if not isinstance(msg, dict):
            return

        status = msg.get("status")

        if status in (
            "IDLE",
            "MOVING",
            "ARRIVED",
            "ERROR"
        ):
            self._etat["status"] = status

        # Arduino confirme un noeud
        if noeud_valide(msg.get("node")):

            nouveau = list(msg["node"])

            a = self._etat["from"]
            b = self._etat["to"]

            if (
                arete_valide(a, b)
                and nouveau == b
            ):

                self._etat["traversed"].append([
                    list(a),
                    list(b)
                ])

            self._etat["node"] = nouveau

            self._etat.update(
                **{
                    "from": None,
                    "to": None
                }
            )

        # Arduino indique une arête en cours
        if (
            status == "MOVING"
            and arete_valide(
                msg.get("from"),
                msg.get("to")
            )
        ):

            self._etat.update(
                **{
                    "from": list(msg["from"]),
                    "to": list(msg["to"])
                }
            )

        elif status in (
            "ARRIVED",
            "IDLE",
            "ERROR"
        ):

            self._etat.update(
                **{
                    "from": None,
                    "to": None
                }
            )

        if isinstance(msg.get("message"), str):

            self._etat["message"] = (
                msg["message"][:200]
            )

    # =====================================
    # LECTURE DES DONNÉES ARDUINO
    # =====================================

    def lire_etat(self):

        # Simulation
        if self.mode == "simulation":

            self._simuler()

            return self._etat_public()

        # Liaison série réelle
        try:

            attendu = self._port.in_waiting

            if attendu:

                self._buffer.extend(
                    self._port.read(
                        min(4096, attendu)
                    )
                )

            # Protection contre une trame trop longue
            if len(self._buffer) > 8192:

                self._buffer.clear()

                raise ValueError(
                    "Trame série trop longue"
                )

            # Lecture des messages JSON
            while b"\n" in self._buffer:

                ligne, _, suite = (
                    self._buffer.partition(b"\n")
                )

                self._buffer = bytearray(suite)

                if not ligne.strip():
                    continue

                try:

                    msg = json.loads(
                        ligne.decode("utf-8")
                    )

                except (ValueError, UnicodeError):
                    continue

                if isinstance(msg, dict):

                    self._integrer_trame(msg)

                    self._dernier_recu = (
                        time.monotonic()
                    )

        except Exception as exc:

            self._etat.update(
                status="ERROR",
                message=f"Erreur série : {exc}"
            )

        return self._etat_public()

    # =====================================
    # FERMETURE DE LA CONNEXION
    # =====================================

    def fermer(self):

        if (
            self._port is not None
            and self._port.is_open
        ):

            self._port.close()
