"""Modèle de carte et itinéraires — sans connexion à un robot."""

from __future__ import annotations

import json
import math
from pathlib import Path

import networkx as nx


def validate_map(data: dict) -> dict:
    """Valide et normalise une carte JSON ; lève ValueError si elle est invalide."""
    if not isinstance(data, dict):
        raise ValueError("La carte doit être un objet JSON.")
    raw_nodes = data.get("nodes")
    raw_edges = data.get("edges")
    if not isinstance(raw_nodes, list) or not raw_nodes:
        raise ValueError("Ajoute au moins un nœud à la carte.")
    if not isinstance(raw_edges, list):
        raise ValueError("Le champ 'edges' doit être une liste.")

    nodes = []
    identifiers = set()
    for index, node in enumerate(raw_nodes, start=1):
        if not isinstance(node, dict):
            raise ValueError(f"Nœud {index} : format invalide.")
        raw_id = node.get("id")
        node_id = "" if raw_id is None or (isinstance(raw_id, float) and not math.isfinite(raw_id)) else str(raw_id).strip()
        if not node_id:
            raise ValueError(f"Nœud {index} : identifiant manquant.")
        if node_id in identifiers:
            raise ValueError(f"Identifiant de nœud en doublon : {node_id}.")
        try:
            x = float(node["x"])
            y = float(node["y"])
        except (KeyError, ValueError, TypeError):
            raise ValueError(f"Nœud {node_id} : coordonnées X/Y invalides.") from None
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"Nœud {node_id} : coordonnées non finies.")
        identifiers.add(node_id)
        nodes.append({"id": node_id, "x": x, "y": y})

    start = str(data.get("start", "")).strip()
    if start not in identifiers:
        raise ValueError("Le point de départ doit appartenir aux nœuds.")

    edges = []
    seen = set()
    for index, edge in enumerate(raw_edges, start=1):
        if not isinstance(edge, dict):
            raise ValueError(f"Arête {index} : format invalide.")
        raw_source, raw_target = edge.get("source"), edge.get("target")
        source = "" if raw_source is None or (isinstance(raw_source, float) and not math.isfinite(raw_source)) else str(raw_source).strip()
        target = "" if raw_target is None or (isinstance(raw_target, float) and not math.isfinite(raw_target)) else str(raw_target).strip()
        if source not in identifiers or target not in identifiers:
            raise ValueError(f"Arête {index} : nœud source ou cible inexistant.")
        if source == target:
            raise ValueError(f"Arête {index} : une liaison doit relier deux nœuds distincts.")
        pair = frozenset((source, target))
        if pair in seen:
            raise ValueError(f"Liaison en doublon entre {source} et {target}.")
        try:
            distance = float(edge["distance"])
        except (KeyError, ValueError, TypeError):
            raise ValueError(f"Arête {index} : distance invalide.") from None
        if not math.isfinite(distance) or distance <= 0:
            raise ValueError(f"Arête {index} : la distance doit être positive.")
        seen.add(pair)
        edges.append({"source": source, "target": target, "distance": distance})

    return {"start": start, "nodes": nodes, "edges": edges}


def read_map(filename: Path) -> dict:
    return validate_map(json.loads(filename.read_text(encoding="utf-8")))


def save_map(filename: Path, data: dict) -> dict:
    normalized = validate_map(data)
    filename.write_text(
        json.dumps(normalized, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return normalized


def to_graph(data: dict) -> nx.Graph:
    graph = nx.Graph()
    graph.add_nodes_from(node["id"] for node in data["nodes"])
    for edge in data["edges"]:
        graph.add_edge(edge["source"], edge["target"], weight=edge["distance"])
    return graph


def shortest_path(data: dict, origin: str, destination: str) -> list[str]:
    graph = to_graph(data)
    try:
        return nx.shortest_path(graph, origin, destination, weight="weight")
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        return []


def path_distance(data: dict, path: list[str]) -> float:
    graph = to_graph(data)
    return sum(graph[a][b]["weight"] for a, b in zip(path, path[1:]))
