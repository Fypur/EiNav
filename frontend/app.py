
import os
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

MODE_LIAISON = os.getenv(
    "ROBOT_MODE", "simulation"
).lower()

PORT = os.getenv(
    "ARDUINO_PORT", "/dev/ttyACM0"
)

DEPART = (0, 0)

# Maillage cartésien 4 x 4
GRAPHE = nx.grid_2d_graph(4, 4)


# =========================================
# LIGNE BLANCHE COURBE
# =========================================

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
    courbe = []

    for i in range(len(controles) - 1):
        p0 = controles[max(i - 1, 0)]
        p1 = controles[i]
        p2 = controles[i + 1]
        p3 = controles[min(i + 2, len(controles) - 1)]

        for j in range(precision):
            t = j / precision
            xy = []

            for a, b, c, d in zip(p0, p1, p2, p3):
                valeur = 0.5 * (
                    2 * b
                    + (-a + c) * t
                    + (2*a - 5*b + 4*c - d) * t**2
                    + (-a + 3*b - 3*c + d) * t**3
                )

                valeur = max(0, min(3, valeur))
                xy.append(valeur)

            courbe.append(tuple(xy))

    return courbe + [controles[-1]]


LIGNE = creer_courbe(CONTROLES_LIGNE)


# =========================================
# COMMUNICATION ARDUINO
# =========================================

if "liaison" not in st.session_state:
    try:
        st.session_state.liaison = CommunicationRobot(
            mode=MODE_LIAISON,
            port=PORT
        )
    except ConnectionError as exc:
        st.error(str(exc))
        st.stop()


# =========================================
# AFFICHAGE DE LA CARTE
# =========================================

def tracer(mode, etat):
    fig = go.Figure()

    if mode == "Navigation sur grille":

        # Affichage des arêtes de la grille
        for a, b in GRAPHE.edges():
            fig.add_trace(go.Scatter(
                x=[a[0], b[0]],
                y=[a[1], b[1]],
                mode="lines",
                line=dict(
                    color="#8293a7",
                    width=3
                ),
                hoverinfo="skip",
                showlegend=False
            ))

        # Arêtes déjà parcourues
        for a, b in etat["traversed"]:

            if GRAPHE.has_edge(tuple(a), tuple(b)):
                fig.add_trace(go.Scatter(
                    x=[a[0], b[0]],
                    y=[a[1], b[1]],
                    mode="lines",
                    line=dict(
                        color="#16a36f",
                        width=6
                    ),
                    hoverinfo="skip",
                    showlegend=False
                ))

        # Arête actuellement empruntée
        a = etat.get("from")
        b = etat.get("to")

        if (
            a is not None
            and b is not None
            and GRAPHE.has_edge(tuple(a), tuple(b))
        ):

            fig.add_trace(go.Scatter(
                x=[a[0], b[0]],
                y=[a[1], b[1]],
                mode="lines",
                line=dict(
                    color="#f2a53c",
                    width=8
                ),
                name="Arête en cours"
            ))

        # Affichage des 16 points
        noeuds = [
            (x, y)
            for y in range(4)
            for x in range(4)
        ]

        fig.add_trace(go.Scatter(
            x=[p[0] for p in noeuds],
            y=[p[1] for p in noeuds],
            mode="markers+text",
            text=[
                f"({x},{y})"
                for x, y in noeuds
            ],
            textposition="top center",
            marker=dict(
                size=12,
                color="#4a8cc0"
            ),
            showlegend=False
        ))

    else:

        # Affichage de la ligne blanche courbe
        fig.add_trace(go.Scatter(
            x=[p[0] for p in LIGNE],
            y=[p[1] for p in LIGNE],
            mode="lines",
            line=dict(
                color="white",
                width=7
            ),
            name="Ligne blanche"
        ))

    # =====================================
    # POSITION DU ROBOT
    # =====================================

    # Position = dernier noeud confirmé
    x, y = etat["node"]

    fig.add_trace(go.Scatter(
        x=[x],
        y=[y],
        mode="markers",
        marker=dict(
            size=22,
            color="#08ad77",
            line=dict(
                color="white",
                width=2
            )
        ),
        name="Dernier noeud confirmé"
    ))

    # =====================================
    # CONFIGURATION DU GRAPHIQUE
    # =====================================

    fig.update_layout(
        height=500,

        margin=dict(
            l=15,
            r=15,
            t=15,
            b=15
        ),

        xaxis=dict(
            title="X",
            range=[-0.4, 3.4],
            dtick=1
        ),

        # Origine (0,0) en haut à gauche
        yaxis=dict(
            title="Y",
            range=[3.4, -0.4],
            dtick=1,
            scaleanchor="x"
        ),

        plot_bgcolor=(
            "#243343"
            if mode == "Suivi d'une ligne blanche"
            else None
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
    f"Communication : {MODE_LIAISON}"
)


@st.fragment(run_every="0.25s")
def interface():

    # Lecture de l'état du robot
    etat = st.session_state.liaison.lire_etat()

    # =====================================
    # CHOIX DU MODE
    # =====================================

    mode = st.radio(
        "Mode de déplacement",
        [
            "Navigation sur grille",
            "Suivi d'une ligne blanche"
        ],
        horizontal=True,
        disabled=etat["status"] in ("MOVING", "SENT")
    )

    carte, actions = st.columns([2, 1])

    # =====================================
    # CARTE
    # =====================================

    with carte:
        tracer(mode, etat)

    # =====================================
    # COMMANDES
    # =====================================

    with actions:

        st.subheader("Commandes")

        # Saisie des coordonnées
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

        else:

            st.info(
                "Mode suivi de ligne blanche. "
                "La position entre deux repères "
                "est inconnue."
            )

        # =================================
        # BOUTON DÉMARRAGE
        # =================================

        texte_bouton = (
            "▶ Partir à la destination"
            if mode == "Navigation sur grille"
            else "▶ Suivre la ligne blanche"
        )

        if st.button(
            texte_bouton,
            type="primary",
            use_container_width=True,
            disabled=etat["status"] in ("MOVING", "SENT")
        ):

            try:

                if mode == "Navigation sur grille":

                    st.session_state.liaison.partir_destination(
                        int(x),
                        int(y)
                    )

                else:

                    st.session_state.liaison.demarrer_suivi_ligne()

                st.rerun(scope="fragment")

            except (ConnectionError, ValueError) as exc:
                st.error(str(exc))

        # =================================
        # RETOUR AU DÉPART
        # =================================

        if st.button(
            "↩ Retour au départ",
            use_container_width=True
        ):

            try:

                st.session_state.liaison.retour_depart()

                st.rerun(scope="fragment")

            except ConnectionError as exc:
                st.error(str(exc))

        # =================================
        # ARRÊT
        # =================================

        if st.button(
            "■ STOP",
            use_container_width=True
        ):

            try:

                st.session_state.liaison.stop()

                st.rerun(scope="fragment")

            except ConnectionError as exc:
                st.error(str(exc))

        # =================================
        # ÉTAT DU ROBOT
        # =================================

        st.divider()

        st.subheader("État du robot")

        st.write(
            "Dernier noeud confirmé :",
            tuple(etat["node"])
        )

        # Afficher l'arête active
        if etat.get("from") is not None:

            st.info(
                f"🔄 Arête en cours : "
                f"{tuple(etat['from'])} → "
                f"{tuple(etat['to'])}"
            )

        elif etat["status"] == "MOVING":

            st.info(
                "🔄 Robot en déplacement"
            )

        # Indicateur d'arrivée
        if etat["status"] == "ARRIVED":

            if MODE_LIAISON == "serie":
                st.success("✅ Arrivée confirmée")
            else:
                st.success("✅ Arrivée simulée")

        elif etat["status"] == "SENT":

            st.info(
                "📨 Commande envoyée à Arduino"
            )

        elif etat["status"] == "ERROR":

            st.error(
                etat.get(
                    "message",
                    "Erreur de communication"
                )
            )

        elif etat["status"] == "IDLE":

            st.warning("⏳ En attente")

        # Vérification de la communication
        if MODE_LIAISON == "serie":

            age = etat["age_s"]

            if age is None or age > 2:

                st.error(
                    "⚠️ Aucune télémétrie récente"
                )

            else:

                st.caption(
                    f"Dernier message reçu : "
                    f"il y a {age:.1f} s"
                )

        st.caption(
            "Dernière commande : "
            f"{st.session_state.liaison.derniere_commande}"
        )


# =========================================
# LANCEMENT
# =========================================

interface()
