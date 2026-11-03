# import rclpy
# from robot.steady_node import SteadyNode


# class Navigation(SteadyNode):

#     def __init__(self):
#         super().__init__('navigation')


# def main():
#     rclpy.init()
#     node = Navigation()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()


from collections import deque

class IntersectionPoint:
    def __init__(self, x, y, left: bool, right: bool, top: bool, bottom: bool):
        self.x = x
        self.y = y
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom

    def __repr__(self):
        r = ""
        if self.top :
            r += " x \n"
        if self.left:
            r += "x"
        else:
            r += " "
        r += "o"
        if self.right:
            r += "x\n"
        else:
            r += " \n"
        if self.bottom:
            r += " x "
        return r

class Grid:
    def __init__(self, size=4):
        self.size = size
        self.grid = [[IntersectionPoint(i, j, True, True, True, True) for i in range(size)] for j in range(size)]
        for i in range(size):
            self.grid[0][i].top = False
            self.grid[size-1][i].bottom = False
            self.grid[i][0].left = False
            self.grid[i][size-1].right = False

class Navigation:
    def __init__(self):
        super().__init__() if hasattr(super(), '__init__') else None

    def generate_path(self, grid, start, end, robot_direction):
        dir_vectors = {
            "N": (0, -1),
            "S": (0, 1),
            "E": (1, 0),
            "W": (-1, 0)
        }
        
        current_dir = robot_direction
        
        def get_prioritized_neighbors(x, y, current_dir):
            point = grid.grid[y][x]
            size = grid.size
            
            # Liste de tous les mouvements possibles
            candidates = []
            if point.top and y > 0:
                candidates.append(("N", (x, y - 1)))
            if point.bottom and y < size - 1:
                candidates.append(("S", (x, y + 1)))
            if point.left and x > 0:
                candidates.append(("W", (x - 1, y)))
            if point.right and x < size - 1:
                candidates.append(("E", (x + 1, y)))
                
            # On trie les candidats pour mettre en premier celui qui garde la même direction
            # puis, on peut ordonner par distance euclidienne vers la fin pour guider le BFS
            def sort_key(item):
                d, (nx, ny) = item
                is_straight = (d == current_dir) # 0 si tt droit 1 sinon
                straight_penalty = 0 if is_straight else 1
                dist_to_end = abs(nx - end[0]) + abs(ny - end[1])
                return (straight_penalty, dist_to_end)

            candidates.sort(key=sort_key)
            return [n for _, n in candidates]

        # bfs
        queue = deque([([start], robot_direction)])
        visited = {start}
        path = []

        while queue:
            current_path, curr_dir = queue.popleft()
            current = current_path[-1]
            
            if current == end:
                path = current_path
                break
                
            x, y = current
            neighbors = get_prioritized_neighbors(x, y, curr_dir)
            
            for n in neighbors:
                if n not in visited:
                    visited.add(n)
                    # Déterminer la nouvelle direction si on va vers ce voisin
                    nx, ny = n
                    if nx > x: new_d = "E"
                    elif nx < x: new_d = "W"
                    elif ny > y: new_d = "S"
                    else: new_d = "N"
                    
                    queue.append((current_path + [n], new_d))

        if not path:
            print("Aucun chemin trouvé !")
            return []

        print(f"Chemin (coordonnées) : {path}")

        # Association des flèches
        path_directions = {}
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i+1]
            if x2 > x1: path_directions[(x1, y1)] = '>'
            elif x2 < x1: path_directions[(x1, y1)] = '<'
            elif y2 > y1: path_directions[(x1, y1)] = 'v'
            elif y2 < y1: path_directions[(x1, y1)] = '^'
        
        if len(path) > 1:
            path_directions[path[-1]] = path_directions[path[-2]]
        else:
            dir_map = {"N": "^", "S": "v", "E": ">", "W": "<"}
            path_directions[path[0]] = dir_map.get(robot_direction, 'o')

        # Affichage visuel
        print("\nAffichage du chemin (grille) :")
        for y in range(grid.size):
            row_str = ""
            for x in range(grid.size):
                if (x, y) in path_directions:
                    row_str += f" {path_directions[(x, y)]} "
                else:
                    row_str += " . "
            print(row_str)

        return path, path_directions

if __name__ == '__main__':
    grid = Grid(4)
    nav = Navigation()
    
    start_pos = (0, 0)
    end_pos = (2, 2)
    robot_dir = "E"
    
    nav.generate_path(grid, start_pos, end_pos, robot_dir)
