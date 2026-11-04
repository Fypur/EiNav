"""Tableau de bord Streamlit.

L'onglet « Robot » commande le robot réel (livraison ou parcours en 8) et l'affiche en direct.
Les autres onglets sont une simulation graphique : ils n'envoient aucune commande au robot.
"""

from __future__ import annotations

import html
import json
from datetime import datetime

import pandas as pd
import plotly.graph_objects as go
import streamlit as st

from ligne import line_length, position_at, read_line, sample_line, save_line, validate_line
from navigation import path_distance, read_map, save_map, shortest_path, validate_map
from utils import (
    CAMERA_STREAM_PORT,
    LINE_FILE,
    LIVE_PAGE_HEIGHT,
    MAP_FILE,
    MODE_LINE,
    MODE_NAV,
    TELEMETRY_PORT,
)

st.set_page_config(
    page_title="Robot | Poste de pilotage",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown(
    """
    <style>
    .block-container { padding-top: 1.4rem; max-width: 1500px; }
    [data-testid="stMetric"] {
        border: 1px solid rgba(148,163,184,.17);
        background-color: rgba(148,163,184,.06);
        border-radius: 13px;
        padding: 13px 17px;
    }
    .dashboard-kicker { color:#38BDF8; text-transform:uppercase;
        letter-spacing:.19em; font-size:.78rem; font-weight:700; margin-bottom:.25rem; }
    .dashboard-note { color:#9EB0C6; margin-bottom:.5rem; }
    .camera-fallback { padding: 3.4rem 1rem; border-radius: 13px;
        background: repeating-linear-gradient(45deg, #121d2d 0, #121d2d 12px,
        #172438 12px, #172438 24px); border: 1px solid #334155;
        text-align:center; color:#cbd5e1; }
    .camera-icon { font-size: 2.5rem; }
    .camera-full { position: relative; margin-bottom: 1.2rem; }
    .camera-full img { display:block; width:100%; height:calc(100vh - 4rem);
        object-fit:contain; image-rendering:pixelated; background:#000;
        border-radius:13px; border:1px solid #334155; }
    .camera-full a { position:absolute; top:.6rem; right:.8rem; padding:.25rem .6rem;
        border-radius:8px; background:rgba(15,23,42,.75); color:#e2e8f0;
        text-decoration:none; font-size:.85rem; }
    </style>
    """,
    unsafe_allow_html=True,
)


def robot_host() -> str:
    """Machine qui sert l'interface (le Raspberry Pi), telle que le navigateur la connaît."""
    host = st.context.headers.get("Host", "localhost")
    return host.rsplit(":", 1)[0] if not host.endswith("]") else host


def default_stream_url() -> str:
    return f"http://{robot_host()}:{CAMERA_STREAM_PORT}/stream.mjpg"


def log(message: str) -> None:
    st.session_state.history.insert(
        0, {"heure": datetime.now().strftime("%H:%M:%S"), "événement": message}
    )
    st.session_state.history = st.session_state.history[:50]


def initialize() -> None:
    if "map_data" not in st.session_state:
        st.session_state.map_data = read_map(MAP_FILE)
    if "line_data" not in st.session_state:
        st.session_state.line_data = read_line(LINE_FILE)
    data = st.session_state.map_data
    ids = [node["id"] for node in data["nodes"]]
    if st.session_state.get("position") not in ids:
        st.session_state.position = data["start"]
    if st.session_state.get("destination_select") not in ids:
        st.session_state.destination_select = ids[-1]
    if "route" not in st.session_state:
        st.session_state.route = []
    if "status" not in st.session_state:
        st.session_state.status = "Prêt"
    if "history" not in st.session_state:
        st.session_state.history = [
            {"heure": datetime.now().strftime("%H:%M:%S"),
             "événement": "Interface démarrée en mode démonstration"}
        ]
    if "mode" not in st.session_state:
        st.session_state.mode = MODE_NAV
    if "line_progress" not in st.session_state:
        st.session_state.line_progress = 0
    if "show_camera" not in st.session_state:
        st.session_state.show_camera = False
    # Après une sauvegarde, mettre le slider à zéro AVANT d'instancier le widget.
    if st.session_state.pop("reset_line_next", False):
        st.session_state.line_progress = 0


def plan(destination: str, *, returning: bool = False) -> None:
    origin = st.session_state.position
    route = shortest_path(st.session_state.map_data, origin, destination)
    if not route:
        st.session_state.route = []
        st.session_state.status = "Chemin introuvable"
        log(f"Aucun itinéraire entre {origin} et {destination}")
        return
    st.session_state.route = route
    st.session_state.status = "Retour prévu" if returning else "Trajet prévu"
    log(f"{'Retour' if returning else 'Trajet'} préparé : {' → '.join(route)}")


def advance_one_node() -> None:
    route = st.session_state.route
    if len(route) < 2:
        return
    st.session_state.position = route[1]
    st.session_state.route = route[1:]
    log(f"Déplacement simulé vers {st.session_state.position}")
    if len(st.session_state.route) == 1:
        st.session_state.status = "Arrivé (simulation)"
        log("Destination atteinte sur le maillage")
    else:
        st.session_state.status = "En déplacement (simulation)"


def advance_line(delta: int) -> None:
    # Callback exécuté AVANT le rerun : autorise la modification du slider.
    old = int(st.session_state.line_progress)
    st.session_state.line_progress = max(0, min(100, old + delta))
    if st.session_state.line_progress == 100:
        st.session_state.status = "Fin de ligne atteinte"
    else:
        st.session_state.status = "Suivi de ligne (simulation)"
    log(f"Avancement sur ligne blanche : {st.session_state.line_progress} %")


def reset_line() -> None:
    # Callback exécuté avant le widget slider sur le prochain rerun.
    st.session_state.line_progress = 0
    st.session_state.status = "Début de ligne (simulation)"
    log("Robot simulé replacé au début de la ligne")


def cartesian_layout(fig: go.Figure, xs: list[float], ys: list[float]) -> None:
    all_x = xs if xs else [0, 3]
    all_y = ys if ys else [0, 3]
    margin_x = max(0.55, (max(all_x) - min(all_x)) * 0.11)
    margin_y = max(0.55, (max(all_y) - min(all_y)) * 0.11)
    fig.update_layout(
        height=465,
        margin=dict(l=18, r=18, t=12, b=16),
        paper_bgcolor="#101B2D", plot_bgcolor="#101B2D",
        font=dict(color="#CBD5E1"),
        showlegend=False,
        xaxis=dict(
            title="X", title_font=dict(size=12),
            range=[min(all_x) - margin_x, max(all_x) + margin_x],
            tickmode="linear", dtick=1, showgrid=True, gridcolor="#27374D",
            zeroline=True, zerolinecolor="#73849B", zerolinewidth=2,
            fixedrange=True,
        ),
        yaxis=dict(
            title="Y", title_font=dict(size=12),
            range=[min(all_y) - margin_y, max(all_y) + margin_y],
            tickmode="linear", dtick=1, showgrid=True, gridcolor="#27374D",
            zeroline=True, zerolinecolor="#73849B", zerolinewidth=2,
            scaleanchor="x", scaleratio=1, fixedrange=True,
        ),
    )


def make_map(data: dict, position: str, destination: str, route: list[str]) -> go.Figure:
    coords = {node["id"]: (node["x"], node["y"]) for node in data["nodes"]}
    ids = [node["id"] for node in data["nodes"]]
    fig = go.Figure()
    for edge in data["edges"]:
        a, b = coords[edge["source"]], coords[edge["target"]]
        fig.add_trace(go.Scatter(
            x=[a[0], b[0]], y=[a[1], b[1]], mode="lines",
            line=dict(color="#33465F", width=4),
            hoverinfo="skip", showlegend=False,
        ))
    if len(route) > 1:
        fig.add_trace(go.Scatter(
            x=[coords[p][0] for p in route],
            y=[coords[p][1] for p in route], mode="lines",
            line=dict(color="#38BDF8", width=7),
            hoverinfo="skip", showlegend=False,
        ))
    colors = [
        "#10B981" if node_id == position else
        "#F59E0B" if node_id == destination else
        "#687D99" for node_id in ids
    ]
    fig.add_trace(go.Scatter(
        x=[coords[p][0] for p in ids], y=[coords[p][1] for p in ids],
        mode="markers+text", text=ids, textposition="middle center",
        textfont=dict(color="#FFFFFF", size=12, family="Arial Black"),
        marker=dict(color=colors, size=[34 if p == position else 31 for p in ids],
                    line=dict(color="#E2E8F0", width=1.5)),
        customdata=[[p] for p in ids],
        hovertemplate="Point %{customdata[0]} — X=%{x}, Y=%{y}<extra></extra>",
        selected=dict(marker=dict(opacity=1)),
        unselected=dict(marker=dict(opacity=1)),
    ))
    cartesian_layout(fig, [x for x, _ in coords.values()], [y for _, y in coords.values()])
    fig.update_layout(dragmode="select", clickmode="event+select")
    return fig


def make_line_plot(line_data: dict, grid_data: dict, progress: int) -> go.Figure:
    samples = sample_line(line_data)
    robot_x, robot_y = position_at(samples, progress / 100)
    waypoints = line_data["points"]
    fig = go.Figure()

    # 16 points de référence cartésiens, indépendants de la trajectoire blanche.
    nodes = grid_data["nodes"]
    fig.add_trace(go.Scatter(
        x=[n["x"] for n in nodes], y=[n["y"] for n in nodes],
        mode="markers+text", text=[n["id"] for n in nodes],
        textposition="top right", textfont=dict(size=10, color="#73849B"),
        marker=dict(size=7, color="#40536C"),
        hovertemplate="Repère %{text} (%{x}, %{y})<extra></extra>",
    ))
    # La ligne blanche suit une spline ou des segments selon la configuration.
    fig.add_trace(go.Scatter(
        x=[p[0] for p in samples], y=[p[1] for p in samples],
        mode="lines", line=dict(color="#FFFFFF", width=8),
        name="Ligne blanche", hoverinfo="skip",
    ))
    if progress > 0:
        # Représentation du morceau parcouru, sur le même tracé.
        total = line_length(samples)
        traveled = total * progress / 100
        traveled_points = [samples[0]]
        distance = 0.0
        for a, b in zip(samples, samples[1:]):
            span = ((b[0] - a[0])**2 + (b[1] - a[1])**2) ** 0.5
            if distance + span >= traveled:
                traveled_points.append((robot_x, robot_y))
                break
            traveled_points.append(b)
            distance += span
        fig.add_trace(go.Scatter(
            x=[p[0] for p in traveled_points], y=[p[1] for p in traveled_points],
            mode="lines", line=dict(color="#38BDF8", width=5),
            hoverinfo="skip", name="Partie parcourue",
        ))
    fig.add_trace(go.Scatter(
        x=[p["x"] for p in waypoints], y=[p["y"] for p in waypoints],
        mode="markers", marker=dict(color="#F59E0B", size=9,
                                    line=dict(color="#1F2937", width=1)),
        customdata=list(range(1, len(waypoints)+1)),
        hovertemplate="Point de contrôle %{customdata}<br>X=%{x:.2f}, Y=%{y:.2f}<extra></extra>",
    ))
    fig.add_trace(go.Scatter(
        x=[robot_x], y=[robot_y], mode="markers",
        marker=dict(size=22, color="#10B981", line=dict(color="#D1FAE5", width=3)),
        hovertemplate="Robot simulé<br>X=%{x:.2f}, Y=%{y:.2f}<extra></extra>",
    ))
    all_x = [n["x"] for n in nodes] + [p[0] for p in samples]
    all_y = [n["y"] for n in nodes] + [p[1] for p in samples]
    cartesian_layout(fig, all_x, all_y)
    return fig


initialize()
data = st.session_state.map_data
line_data = st.session_state.line_data
ids = [node["id"] for node in data["nodes"]]

with st.sidebar:
    st.markdown("### 🤖 Robot Control")
    st.caption("Tableau de bord local · Prototype V2")
    st.divider()
    st.subheader("Mode de déplacement")
    mode = st.radio("Sélectionne un mode", [MODE_NAV, MODE_LINE], key="mode")
    st.caption("Le mode ligne blanche accepte les courbes et n'impose pas de suivre les arêtes du maillage.")
    st.divider()
    st.subheader("Modules")
    show_camera = st.toggle("Afficher la caméra", key="show_camera")
    if show_camera:
        stream_url = st.text_input("Adresse du flux", value=default_stream_url(), key="camera_url")
        st.caption(
            "Flux MJPEG diffusé par camera_pi.py. Si l'image reste vide, vérifier que le "
            "programme du robot tourne et que l'adresse est celle du Raspberry Pi."
        )
    st.caption("Seul l'onglet « Robot » agit sur le robot réel : ce choix de mode ne concerne que la simulation.")
    st.divider()
    st.subheader("Réseau")
    st.write(f"**Robot :** {robot_host()}")
    st.write(f"**Grille en direct :** port {TELEMETRY_PORT}")
    st.write(f"**Caméra :** flux MJPEG, port {CAMERA_STREAM_PORT}")

if show_camera:
    # Flux caméra sur toute la largeur et toute la hauteur de l'écran, sans déformation ;
    # le tableau de bord reste accessible en faisant défiler la page vers le bas.
    safe_url = html.escape(stream_url, quote=True)
    page_url = html.escape(stream_url.rsplit("/", 1)[0] + "/", quote=True)
    st.markdown(
        '<style>.block-container { max-width: 100%; }</style>'
        f'<div class="camera-full">'
        f'<img src="{safe_url}" alt="Flux caméra indisponible">'
        f'<a href="{page_url}" target="_blank" rel="noopener">⛶ plein écran</a></div>',
        unsafe_allow_html=True,
    )

st.markdown('<div class="dashboard-kicker">Station de pilotage • Prototype V2</div>', unsafe_allow_html=True)
st.title("Tableau de bord du robot")
st.markdown(
    '<div class="dashboard-note">Maillage cartésien 4 × 4 et suivi de ligne blanche courbe</div>',
    unsafe_allow_html=True,
)

live_tab, nav_tab, edit_tab, line_tab, log_tab = st.tabs([
    "🤖 Robot", "🗺️ Pilotage (simulation)", "🧩 Édition du maillage",
    "〰️ Édition de la ligne", "📋 Journal",
])

with live_tab:
    # Page servie par le programme du robot : elle reste à jour sans relancer Streamlit.
    st.iframe(f"http://{robot_host()}:{TELEMETRY_PORT}/", height=LIVE_PAGE_HEIGHT)
    st.caption("Si la page reste vide, vérifier que le programme du robot (EiNav.py) tourne.")

with nav_tab:
    st.info("🧪 SIMULATION — aucune commande n’est envoyée au robot réel.")
    m1, m2, m3, m4 = st.columns(4)
if mode == MODE_NAV:
    coords = {n["id"]: (n["x"], n["y"]) for n in data["nodes"]}
    px, py = coords[st.session_state.position]
    m1.metric("Position simulée", f"{st.session_state.position} ({px:g}, {py:g})")
    m2.metric("Destination", st.session_state.destination_select)
    m3.metric("État", st.session_state.status)
    m4.metric("Points du maillage", len(ids))
else:
    samples = sample_line(line_data)
    lx, ly = position_at(samples, st.session_state.line_progress / 100)
    m1.metric("Position simulée", f"({lx:.2f}, {ly:.2f})")
    m2.metric("Progression", f"{st.session_state.line_progress} %")
    m3.metric("Longueur de ligne", f"{line_length(samples):.2f} u")
    m4.metric("Mode", "Ligne blanche")

with nav_tab:
    left, right = st.columns([1.8, 1], gap="large")
    with left:
        if mode == MODE_NAV:
            st.subheader("Carte cartésienne — 16 points")
            st.caption("Abscisse X et ordonnée Y : clique sur un point pour choisir la destination.")
            event = st.plotly_chart(
                make_map(data, st.session_state.position,
                         st.session_state.destination_select, st.session_state.route),
                key="robot_map_plot", on_select="rerun", selection_mode="points",
                config={"displaylogo": False, "scrollZoom": False},
                width="stretch",
            )
            for point in event.selection.points[-1:]:
                raw_id = point.get("customdata")
                selected_id = raw_id[0] if isinstance(raw_id, (list, tuple)) else raw_id
                marker = (point.get("curve_number"), point.get("point_index"), selected_id)
                if selected_id in ids and marker != st.session_state.get("last_map_selection"):
                    st.session_state.destination_select = selected_id
                    st.session_state.last_map_selection = marker
                    st.rerun()
            st.caption("🟢 Robot simulé   🟠 Destination   🔵 Chemin calculé")
            if st.session_state.route:
                st.markdown("**Itinéraire :** " + " → ".join(st.session_state.route))
                st.caption(
                    f"Distance restante : {path_distance(data, st.session_state.route):g} unité(s) de carte"
                )
            else:
                st.caption("Aucun itinéraire préparé.")
        else:
            st.subheader("Ligne blanche — trajectoire libre")
            st.caption("La ligne peut être courbée ; sa forme est définie par des points de contrôle X/Y.")
            st.plotly_chart(
                make_line_plot(line_data, data, st.session_state.line_progress),
                config={"displaylogo": False, "scrollZoom": False},
                width="stretch", key="line_plot",
            )
            st.caption("⚪ Ligne à suivre   🟠 Points de contrôle   🟢 Robot simulé   🔵 Portion parcourue")
            st.progress(st.session_state.line_progress / 100)
    with right:
        st.subheader("Commandes")
        if mode == MODE_NAV:
            destination = st.selectbox("Choisir une destination", ids, key="destination_select")
            if st.button("📍 Préparer le trajet", type="primary", use_container_width=True):
                plan(destination)
                st.rerun()
            if st.button("🏠 Retour au point de départ", use_container_width=True):
                plan(data["start"], returning=True)
                st.rerun()
            if st.button("➡️ Avancer d'un point (simulation)", use_container_width=True,
                         disabled=len(st.session_state.route) < 2):
                advance_one_node()
                st.rerun()
        else:
            st.markdown("**Suivi de la ligne blanche**")
            st.caption("Déplacement virtuel le long de la courbe, indépendant des arêtes du maillage.")
            st.slider("Position le long de la ligne (%)", 0, 100, key="line_progress")
            st.button("▶️ Avancer de 5 % (simulation)", type="primary",
                      use_container_width=True,
                      disabled=st.session_state.line_progress >= 100,
                      on_click=advance_line, args=(5,))
            st.button("↩️ Revenir au début de la ligne", use_container_width=True,
                      on_click=reset_line)
            st.caption("La progression est manuelle : aucun capteur n'est simulé, aucun robot ne bouge.")
        if st.button("🛑 STOP (simulation)", use_container_width=True):
            if mode == MODE_NAV:
                st.session_state.route = []
            st.session_state.status = "Arrêt (simulation)"
            log("STOP demandé dans l'interface")
            st.rerun()
        st.caption("Ces commandes seront raccordées à l'Arduino ultérieurement.")

with edit_tab:
    st.subheader("Éditer le maillage cartésien (4 × 4)")
    st.write(
        "Le maillage possède **16 nœuds**, initialement sur les coordonnées X et Y de 0 à 3. "
        "Tu peux changer leurs coordonnées et leurs liaisons. Les 16 identifiants restent présents."
    )
    with st.form("map_editor"):
        st.markdown("**1. Les 16 nœuds (X, Y)**")
        df_nodes = st.data_editor(
            pd.DataFrame(data["nodes"]), num_rows="fixed", hide_index=True,
            width="stretch",
            column_config={
                "id": st.column_config.TextColumn("Identifiant", required=True),
                "x": st.column_config.NumberColumn("X", required=True, format="%.2f"),
                "y": st.column_config.NumberColumn("Y", required=True, format="%.2f"),
            },
        )
        st.markdown("**2. Arêtes (liaisons autorisées)**")
        df_edges = st.data_editor(
            pd.DataFrame(data["edges"]), num_rows="dynamic", hide_index=True,
            width="stretch",
            column_config={
                "source": st.column_config.TextColumn("De", required=True),
                "target": st.column_config.TextColumn("Vers", required=True),
                "distance": st.column_config.NumberColumn(
                    "Distance", required=True, min_value=0.01, format="%.2f"
                ),
            },
        )
        start_id = st.selectbox("Point de départ", ids, index=ids.index(data["start"]))
        do_save = st.form_submit_button("💾 Enregistrer le maillage", type="primary")

    if do_save:
        try:
            candidate = {
                "start": start_id,
                "nodes": df_nodes.to_dict(orient="records"),
                "edges": df_edges.to_dict(orient="records"),
            }
            if len(candidate["nodes"]) != 16:
                raise ValueError("Le maillage doit contenir exactement 16 nœuds.")
            saved = save_map(MAP_FILE, validate_map(candidate))
        except (ValueError, TypeError, OSError) as exc:
            st.error(f"Impossible d'enregistrer : {exc}")
        else:
            st.session_state.map_data = saved
            new_ids = {node["id"] for node in saved["nodes"]}
            if st.session_state.position not in new_ids:
                st.session_state.position = saved["start"]
            # La destination (clé d'un widget Streamlit) est réconciliée
            # au début du prochain rerun, avant la création du selectbox.
            st.session_state.route = []
            st.session_state.status = "Maillage mis à jour"
            st.session_state.pop("last_map_selection", None)
            log("Maillage de 16 points enregistré dans carte.json")
            st.success("Maillage enregistré.")
            st.rerun()

    st.download_button(
        "⬇️ Télécharger le maillage JSON",
        data=json.dumps(data, indent=2, ensure_ascii=False),
        file_name="carte.json", mime="application/json",
    )
    st.caption("Les changements sont enregistrés sur le Raspberry Pi dans carte.json.")

with line_tab:
    st.subheader("Créer une ligne blanche, droite ou courbe")
    st.write(
        "Modifie les **points de contrôle X/Y** dans l'ordre de parcours. "
        "La courbe lisse passe par ces points ; une ligne polygonale est aussi possible. "
        "Ce tracé est indépendant des 16 nœuds et des arêtes du maillage."
    )
    with st.form("line_editor"):
        df_line = st.data_editor(
            pd.DataFrame(line_data["points"]), num_rows="dynamic", hide_index=True,
            width="stretch",
            column_config={
                "x": st.column_config.NumberColumn("X", required=True, format="%.2f"),
                "y": st.column_config.NumberColumn("Y", required=True, format="%.2f"),
            },
        )
        smooth = st.checkbox("Courbe lisse (Catmull-Rom)", value=line_data["smooth"])
        save_white_line = st.form_submit_button("💾 Enregistrer la ligne", type="primary")
    if save_white_line:
        try:
            saved_line = save_line(
                LINE_FILE,
                validate_line({"smooth": smooth, "points": df_line.to_dict(orient="records")}),
            )
        except (ValueError, TypeError, OSError) as exc:
            st.error(f"Impossible d'enregistrer : {exc}")
        else:
            st.session_state.line_data = saved_line
            st.session_state.reset_line_next = True
            st.session_state.status = "Ligne blanche mise à jour"
            log("Tracé de ligne blanche enregistré dans ligne_blanche.json")
            st.success("Ligne blanche enregistrée ; position remise au début.")
            st.rerun()
    st.download_button(
        "⬇️ Télécharger la ligne JSON",
        data=json.dumps(line_data, indent=2, ensure_ascii=False),
        file_name="ligne_blanche.json", mime="application/json",
    )
    st.caption(
        "Les points X/Y décrivent ici la géométrie à afficher, pas un algorithme de lecture des capteurs. "
        "Le suivi réel d'une ligne blanche courbe sera effectué plus tard par l'Arduino et ses capteurs."
    )

with log_tab:
    st.subheader("Historique des interactions")
    st.dataframe(pd.DataFrame(st.session_state.history), hide_index=True, width="stretch")
    st.caption("Journal local à cette session. Aucun ordre n'est envoyé à un robot.")
