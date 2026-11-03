
import os
import time

import networkx as nx
import plotly.graph_objects as go
import streamlit as st

from communication import CommunicationRobot


# =========================================
# CONFIGURATION
# =========================================

st.set_page_config(
    page_title="Contrôle du robot",
    page_icon="🤖",
    layout="wide"
)

st.title("🤖 Contrôle du robot")

# Modes disponibles :
# simulation : sans Arduino
# serie      : avec Arduino connecté

ROBOT_MODE = os.getenv(
    "ROBOT_MODE", "simulation"
).lower()

ARDUINO_PORT = os.getenv(
    "ARDUINO_PORT", "/dev/ttyACM0"
)

# Point de départ : EN HAUT À GAUCHE
DEPART = (0, 0)

# Grille de 16 points (4 x 4)
GRAPHE = nx.grid_2d_graph(4, 4)


# =========================================
# LIGNE BLANCHE COURBE
# =========================================

# Coordonnées de la ligne blanche
# X augmente vers la droite
# Y augmente vers le bas

CONTROLES_LIGNE = [
    (0, 0),
    (0.4, 0.5),
    (0.8, 1.8),
    (1.5, 2.4),
    (2.1, 1.0),
    (2.6, 1.4),
    (3, 3)
]


def creer_courbe(controles, precision=18):
    """
    Génère une trajectoire courbe
    à partir des points de contrôle.
    """

    courbe = []

    for i in range(len(controles) - 1):

        p0 = controles[max(0, i - 1)]
        p1 = controles[i]
        p2 = controles[i + 1]
        p3 = controles[
            min(i + 2, len(controles) - 1)
        ]

        for k in range(precision):

            t = k / precision
            xy = []

            for a, b, c, d in zip(
                p0, p1, p2, p3
            ):

                valeur = 0.5 * (
                    2 * b
                    + (-a + c) * t
                    + (2*a - 5*b + 4*c - d) * t**2
                    + (-a + 3*b - 3*c + d) * t**3
                )

                valeur = max(
                    0, min(3, valeur)
                )

                xy.append(round(valeur, 3))

            courbe.append(tuple(xy))

    courbe.append(controles[-1])

    return courbe


LIGNE = creer_courbe(CONTROLES_LIGNE)


# =========================================
# COMMUNICATION ARDUINO
# =========================================

if "liaison" not in st.session_state:

    try:

        st.session_state.liaison = (
            CommunicationRobot(
                mode=ROBOT_MODE,
                port=ARDUINO_PORT
            )
        )

    except ConnectionError as exc:

        st.error(
            f"Erreur de connexion Arduino : {exc}"
        )

        st.stop()


# =========================================
# INITIALISATION ROBOT
# =========================================

valeurs_initiales = {

    "position": DEPART,

    "indice_ligne": 0,

    "reste": [],

    "etat": "En attente",

    "mode_courant": "Navigation sur grille",

    "dernier_pas": time.monotonic(),

}

for cle, valeur in valeurs_initiales.items():

    if cle not in st.session_state:

        st.session_state[cle] = valeur


# =========================================
# DÉMARRAGE DU ROBOT (SIMULATION)
# =========================================

def demarrer(etapes):

    st.session_state.reste = list(etapes)

    st.session_state.dernier_pas = (
        time.monotonic()
    )

    if st.session_state.reste:

        st.session_state.etat = (
            "En déplacement"
        )

    else:

        st.session_state.etat = "Arrivé"


# =========================================
# ACTUALISATION DU ROBOT
# =========================================

def synchroniser(mode):

    statut = (
        st.session_state.liaison.lire_etat()
    )

    # -------------------------------------
    # MODE ARDUINO
    # -------------------------------------

    if ROBOT_MODE == "serie":

        etat_recu = statut.get("status")

        if etat_recu in ("SENT", "MOVING"):

            st.session_state.etat = (
                "En déplacement"
            )

        elif etat_recu == "ARRIVED":

            st.session_state.etat = "Arrivé"

            st.session_state.reste = []

        elif etat_recu == "ERROR":

            st.session_state.etat = "Erreur"

        x = statut.get("x")
        y = statut.get("y")

        if (
            isinstance(x, (int, float))
            and isinstance(y, (int, float))
        ):

            st.session_state.position = (x, y)

        return statut

    # -------------------------------------
    # MODE SIMULATION
    # -------------------------------------

    if (
        st.session_state.etat == "En déplacement"
        and st.session_state.reste
    ):

        if mode == "Navigation sur grille":

            duree = 0.8

        else:

            duree = 0.09

        maintenant = time.monotonic()

        nb_pas = min(
            len(st.session_state.reste),
            int(
                (
                    maintenant
                    - st.session_state.dernier_pas
                ) / duree
            )
        )

        if nb_pas > 0:

            for _ in range(nb_pas):

                suivant = (
                    st.session_state.reste.pop(0)
                )

                if mode == "Navigation sur grille":

                    st.session_state.position = suivant

                else:

                    st.session_state.indice_ligne = (
                        suivant
                    )

                    st.session_state.position = (
                        LIGNE[suivant]
                    )

            st.session_state.dernier_pas += (
                nb_pas * duree
            )

            if not st.session_state.reste:

                st.session_state.etat = "Arrivé"

    return statut


# =========================================
# AFFICHAGE DE LA CARTE
# =========================================

def tracer(mode):

    fig = go.Figure()

    # -------------------------------------
    # MODE NAVIGATION SUR GRILLE
    # -------------------------------------

    if mode == "Navigation sur grille":

        # Liaisons entre les points
        for a, b in GRAPHE.edges():

            fig.add_trace(go.Scatter(

                x=[a[0], b[0]],
                y=[a[1], b[1]],

                mode="lines",

                line=dict(
                    color="#8b9cae",
                    width=3
                ),

                showlegend=False,
                hoverinfo="skip"

            ))

        # Coordonnées des 16 points
        coords = [
            (x, y)
            for y in range(4)
            for x in range(4)
        ]

        fig.add_trace(go.Scatter(

            x=[p[0] for p in coords],

            y=[p[1] for p in coords],

            mode="markers+text",

            text=[
                f"({x},{y})"
                for x, y in coords
            ],

            textposition="top center",

            marker=dict(
                size=13,
                color="#4794d0"
            ),

            showlegend=False

        ))

        # Affichage du trajet
        if (
            st.session_state.reste
            and ROBOT_MODE == "simulation"
        ):

            trajet = (
                [st.session_state.position]
                + st.session_state.reste
            )

            fig.add_trace(go.Scatter(

                x=[p[0] for p in trajet],

                y=[p[1] for p in trajet],

                mode="lines",

                line=dict(
                    color="#e6a343",
                    width=6
                ),

                showlegend=False

            ))

    # -------------------------------------
    # MODE SUIVI DE LIGNE BLANCHE
    # -------------------------------------

    else:

        fig.add_trace(go.Scatter(

            x=[p[0] for p in LIGNE],

            y=[p[1] for p in LIGNE],

            mode="lines",

            line=dict(
                color="white",
                width=8
            ),

            name="Ligne blanche"

        ))

    # -------------------------------------
    # AFFICHAGE DU ROBOT
    # -------------------------------------

    px, py = st.session_state.position

    fig.add_trace(go.Scatter(

        x=[px],
        y=[py],

        mode="markers",

        name="Robot",

        marker=dict(
            size=22,
            color="#16b486",
            line=dict(
                color="white",
                width=2
            )
        )

    ))

    # =====================================
    # CONFIGURATION DES AXES
    # =====================================

    fig.update_layout(

        height=500,

        xaxis=dict(
            title="X",
            range=[-0.4, 3.4],
            dtick=1,
            zeroline=False
        ),

        # IMPORTANT :
        # On inverse l'axe Y.
        # (0,0) est donc en HAUT À GAUCHE.
        # Y augmente vers le bas.

        yaxis=dict(
            title="Y",
            range=[3.4, -0.4],
            dtick=1,
            scaleanchor="x",
            zeroline=False
        ),

        plot_bgcolor=(
            "#263646"
            if mode == "Suivi d'une ligne blanche"
            else None
        ),

        margin=dict(
            l=20,
            r=20,
            t=20,
            b=20
        )

    )

    st.plotly_chart(
        fig,
        use_container_width=True
    )


# =========================================
# INTERFACE STREAMLIT
# =========================================

st.caption(
    f"Mode de communication : {ROBOT_MODE}"
)


@st.fragment(run_every="0.25s")
def interface():

    # -------------------------------------
    # CHOIX DU MODE
    # -------------------------------------

    mode = st.radio(
        "Mode de déplacement",
        [
            "Navigation sur grille",
            "Suivi d'une ligne blanche"
        ],
        horizontal=True,
        disabled=(
            st.session_state.etat == "En déplacement"
        )
    )

    # Réinitialisation au changement de mode
    if st.session_state.mode_courant != mode:

        st.session_state.mode_courant = mode

        if ROBOT_MODE == "simulation":

            st.session_state.position = DEPART

            st.session_state.indice_ligne = 0

            st.session_state.reste = []

            st.session_state.etat = "En attente"

    # Mise à jour de la position
    statut = synchroniser(mode)

    # Organisation de la page
    carte, commandes = st.columns([2, 1])

    # =====================================
    # COMMANDES DU ROBOT
    # =====================================

    with commandes:

        st.subheader("Commandes")

        # ---------------------------------
        # SAISIE DES COORDONNÉES
        # ---------------------------------

        if mode == "Navigation sur grille":

            col_x, col_y = st.columns(2)

            x = col_x.number_input(
                "Destination X",
                min_value=0,
                max_value=3,
                value=2,
                step=1
            )

            y = col_y.number_input(
                "Destination Y",
                min_value=0,
                max_value=3,
                value=2,
                step=1
            )

            destination = (
                int(x),
                int(y)
            )

            st.write(
                f"Destination choisie : "
                f"**{destination}**"
            )

        else:

            st.info(
                "Le robot suit une ligne blanche "
                "qui peut être courbe."
            )

        # ---------------------------------
        # BOUTON DÉMARRAGE
        # ---------------------------------

        if mode == "Navigation sur grille":

            texte_bouton = (
                "▶ Partir à la destination"
            )

        else:

            texte_bouton = (
                "▶ Démarrer le suivi de ligne"
            )

        if st.button(
            texte_bouton,
            type="primary",
            use_container_width=True,
            disabled=(
                st.session_state.etat == "En déplacement"
            )
        ):

            try:

                if mode == "Navigation sur grille":

                    # Envoi de la commande
                    st.session_state.liaison.partir_destination(
                        *destination
                    )

                    if ROBOT_MODE == "simulation":

                        chemin = nx.shortest_path(
                            GRAPHE,
                            st.session_state.position,
                            destination
                        )

                        demarrer(chemin[1:])

                    else:

                        st.session_state.etat = (
                            "En déplacement"
                        )

                else:

                    # Démarrage du suivi de ligne
                    st.session_state.liaison.demarrer_suivi_ligne()

                    if ROBOT_MODE == "simulation":

                        demarrer(
                            range(
                                st.session_state.indice_ligne + 1,
                                len(LIGNE)
                            )
                        )

                    else:

                        st.session_state.etat = (
                            "En déplacement"
                        )

                st.rerun(scope="fragment")

            except (ConnectionError, ValueError) as exc:

                st.error(
                    f"Erreur : {exc}"
                )

        # ---------------------------------
        # RETOUR AU POINT DE DÉPART
        # ---------------------------------

        if st.button(
            "↩ Retour au départ",
            use_container_width=True
        ):

            try:

                # Commande envoyée à Arduino
                st.session_state.liaison.retour_depart()

                if ROBOT_MODE == "simulation":

                    if mode == "Navigation sur grille":

                        chemin = nx.shortest_path(
                            GRAPHE,
                            st.session_state.position,
                            DEPART
                        )

                        demarrer(chemin[1:])

                    else:

                        demarrer(
                            range(
                                st.session_state.indice_ligne - 1,
                                -1,
                                -1
                            )
                        )

                else:

                    st.session_state.etat = (
                        "En déplacement"
                    )

                st.rerun(scope="fragment")

            except ConnectionError as exc:

                st.error(
                    f"Erreur de retour : {exc}"
                )

        # ---------------------------------
        # ÉTAT DU ROBOT
        # ---------------------------------

        st.divider()

        st.subheader("État du robot")

        px, py = st.session_state.position

        st.write(
            f"**Position actuelle :** "
            f"({px:.2f}, {py:.2f})"
        )

        if st.session_state.etat == "Arrivé":

            st.success(
                "✅ Robot arrivé à destination"
            )

        elif st.session_state.etat == "En déplacement":

            st.info(
                "🔄 Robot en déplacement..."
            )

        elif st.session_state.etat == "Erreur":

            st.error(
                statut.get(
                    "message",
                    "Erreur de communication"
                )
            )

        else:

            st.warning(
                "⏳ Robot en attente"
            )

        st.caption(
            "Dernière commande : "
            f"{st.session_state.liaison.derniere_commande}"
        )

    # =====================================
    # AFFICHAGE DU MAILLAGE
    # =====================================

    with carte:

        tracer(mode)


# =========================================
# LANCEMENT DE L'APPLICATION
# =========================================

interface()
