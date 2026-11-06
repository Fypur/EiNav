from __future__ import annotations

from collections import deque
from typing import Deque, List, Optional, Tuple

from planning.direction import Direction
from planning.utils import GRID_SIZE

Position = Tuple[int, int]  # (x, y), y croissant vers le sud


class IntersectionPoint:

    def __init__(self, x: int, y: int, left: bool = True, right: bool = True,
                 top: bool = True, bottom: bool = True) -> None:
        self.x = x
        self.y = y
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom

    def __repr__(self) -> str:
        return f"IntersectionPoint({self.x}, {self.y})"


class Grid:

    def __init__(self, size: int = GRID_SIZE) -> None:
        # size représente le nombre de cases
        # size + 1 représente le nombre d'intersections
        self.size = size
        self.nb_nodes = size + 1

        self.grid = [
            [
                IntersectionPoint(x, y)
                for x in range(self.nb_nodes)
            ]
            for y in range(self.nb_nodes)
        ]

        # Limites de la grille
        for i in range(self.nb_nodes):
            self.grid[0][i].top = False
            self.grid[size][i].bottom = False
            self.grid[i][0].left = False
            self.grid[i][size].right = False

        self._blocked = set()

    @property
    def blocked_edges(self) -> List[Tuple[Position, Position]]:
        """Arêtes enlevées du graphe (les 2 noeuds de chaque arête sont triés)."""
        return sorted(self._blocked)

    def _set_edge(self, a: Position, b: Position, is_open: bool) -> None:
        (ax, ay), (bx, by) = sorted((a, b))
        if ax == bx:  # verticale, a est au nord de b
            self.grid[ay][ax].bottom = is_open
            self.grid[by][bx].top = is_open
        else:  # horizontale, a est à l'ouest de b
            self.grid[ay][ax].right = is_open
            self.grid[by][bx].left = is_open

    def block_edge(self, a: Position, b: Position) -> None:
        """Enlève l'arête entre deux noeuds voisins, dans les deux sens."""
        self._set_edge(a, b, False)
        self._blocked.add((min(a, b), max(a, b)))

    def clear_blocked(self) -> None:
        for a, b in self._blocked:
            self._set_edge(a, b, True)
        self._blocked.clear()


class Navigation:

    def __init__(self) -> None:
        pass

    def generate_path(
        self,
        grid: Grid,
        start: Position,
        end: Position,
        robot_direction: Direction,
    ) -> List[Position]:
        nb_nodes = grid.nb_nodes
        max_idx = nb_nodes - 1

        # Vérification des coordonnées
        for name, (x, y) in (
            ("start", start),
            ("end", end)
        ):
            if not (0 <= x <= max_idx and 0 <= y <= max_idx):
                print(f"{name} {(x, y)} hors de la grille")
                return []

        def get_neighbors(
            x: int, y: int, current_dir: Direction
        ) -> List[Tuple[Direction, Position]]:

            point = grid.grid[y][x]

            candidates: List[Tuple[Direction, Position]] = []

            if point.top and y > 0:
                candidates.append(
                    (Direction.NORTH, (x, y - 1))
                )

            if point.bottom and y < max_idx:
                candidates.append(
                    (Direction.SOUTH, (x, y + 1))
                )

            if point.left and x > 0:
                candidates.append(
                    (Direction.WEST, (x - 1, y))
                )

            if point.right and x < max_idx:
                candidates.append(
                    (Direction.EAST, (x + 1, y))
                )

            # Priorité à la continuité de déplacement
            # puis à la proximité de la destination
            def sort_key(item: Tuple[Direction, Position]) -> Tuple[int, int]:
                direction, (nx, ny) = item

                straight_penalty = (
                    0 if direction == current_dir else 1
                )

                distance = (
                    abs(nx - end[0]) +
                    abs(ny - end[1])
                )

                return straight_penalty, distance

            candidates.sort(key=sort_key)

            return candidates

        # BFS
        queue: Deque[Tuple[Position, List[Position], Direction]] = deque()
        queue.append((start, [start], robot_direction))

        visited = {start}

        while queue:

            current, current_path, current_dir = queue.popleft()

            if current == end:
                print(f"Chemin trouvé : {current_path}")
                return current_path

            x, y = current

            neighbors = get_neighbors(x, y, current_dir)

            for new_direction, next_position in neighbors:

                if next_position not in visited:

                    visited.add(next_position)

                    new_path = current_path + [next_position]

                    queue.append(
                        (
                            next_position,
                            new_path,
                            new_direction
                        )
                    )

        print("Aucun chemin trouvé !")

        return []

    def path_to_directions(self, path: List[Position]) -> List[Direction]:

        directions: List[Direction] = []

        for i in range(len(path) - 1):

            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            if x2 > x1:
                directions.append(Direction.EAST)

            elif x2 < x1:
                directions.append(Direction.WEST)

            elif y2 > y1:
                directions.append(Direction.SOUTH)

            elif y2 < y1:
                directions.append(Direction.NORTH)

        return directions

    def display_path(self, grid: Grid, path: List[Position]) -> None:

        if not path:
            print("Aucun chemin à afficher.")
            return

        directions = self.path_to_directions(path)

        path_visual = {}

        for i in range(len(directions)):

            x, y = path[i]

            if directions[i] == Direction.NORTH:
                path_visual[(x, y)] = "^"

            elif directions[i] == Direction.SOUTH:
                path_visual[(x, y)] = "v"

            elif directions[i] == Direction.EAST:
                path_visual[(x, y)] = ">"

            elif directions[i] == Direction.WEST:
                path_visual[(x, y)] = "<"

        # Affichage du point d'arrivée
        path_visual[path[-1]] = "X"

        print("\nAffichage du chemin :")

        for y in range(grid.nb_nodes):

            row = ""

            for x in range(grid.nb_nodes):

                row += f" {path_visual.get((x, y), '.')} "

            print(row)


class Mission:
    """Aller-retour sur un chemin : demi-tour à destination puis retour par le même chemin.

    on_intersection est appelé à chaque arrêt sur un croisement. Le robot démarre sur le premier
    noeud, donc le premier appel se fait au lancement sans attendre la vision.
    """

    def __init__(self, path: List[Position], heading: Optional[Direction] = None) -> None:
        if not path:
            raise ValueError("Une mission nécessite un chemin non vide.")
        self.start = path[0]
        self.destination = path[-1]
        self.route = list(path)
        self.heading = heading  # orientation courante du robot
        self.position: Position = path[0]  # dernier nœud atteint
        self.destination_reached = False  # True une fois à destination
        self.returned_home = False  # True une fois revenu au départ
        self.line_lost = False  # True si on s'arrête car ligne perdue
        self.no_route = False  # True si on s'arrête car plus de chemin
        self.rerouted = False  # True si le chemin a changé à cause d'une arête bloquée
        self._index = 0

    @property
    def next_node(self) -> Optional[Position]:
        """Noeud vers lequel va le robot (None si la mission est finie)."""
        return None if self.returned_home else self.route[self._index]

    @property
    def target(self) -> Position:
        """Là où le robot doit aller : la destination, puis le départ au retour."""
        return self.start if self.destination_reached else self.destination

    def reroute(self, path: List[Position]) -> None:
        """Change le chemin. path doit partir du noeud du prochain on_intersection."""
        self.route = list(path)
        self._index = 0
        self.rerouted = True

    def on_intersection(self) -> Optional[Direction]:
        """Arrivée sur le noeud visé, renvoie la direction à prendre (None si c'est fini)."""
        self.position = self.route[self._index]

        if self._index == len(self.route) - 1 and not self.destination_reached:
            self.destination_reached = True
            # On inverse le chemin, la première direction est donc un demi-tour
            self.route.reverse()
            self._index = 0

        if self._index == len(self.route) - 1:
            self.returned_home = True
            return None

        self._index += 1
        return Navigation().path_to_directions(self.route[self._index - 1:self._index + 1])[0]


if __name__ == "__main__":

    grid = Grid()
    nav = Navigation()

    start_pos = (0, 1)
    end_pos = (4, 4)

    robot_dir = Direction.SOUTH

    path = nav.generate_path(
        grid,
        start_pos,
        end_pos,
        robot_dir
    )

    directions = nav.path_to_directions(path)

    print("Coordonnées :", path)
    print("Directions :", directions)

    nav.display_path(grid, path)
