
from collections import deque
from direction import Direction


class IntersectionPoint:

    def __init__(self, x, y, left=True, right=True,
                 top=True, bottom=True):
        self.x = x
        self.y = y
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom

    def __repr__(self):
        return f"IntersectionPoint({self.x}, {self.y})"


class Grid:

    def __init__(self, size=4):
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


class Navigation:

    def __init__(self):
        pass

    def generate_path(self, grid, start, end, robot_direction):
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

        def get_neighbors(x, y, current_dir):

            point = grid.grid[y][x]

            candidates = []

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
            def sort_key(item):
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
        queue = deque()
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

    def path_to_directions(self, path):

        directions = []

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

    def display_path(self, grid, path):

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


if __name__ == "__main__":

    grid = Grid(4)
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
