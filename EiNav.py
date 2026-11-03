
from control import Control
from serialpi import Serial
from image_processor import ImageProcessor
from navigation import Grid, Navigation
from robot_state import RobotState
from mode import Mode
from direction import Direction


# Simulation flag
SIMULATION = False


def create_environment(simulation: bool):

    if simulation:
        from webots_connection import WebotsConnection

        connection = WebotsConnection()
        serial = Serial(robot=connection.robot)

    else:
        from real_robot_connection import RealRobotConnection

        connection = RealRobotConnection()
        serial = Serial(robot=None)

    return connection, serial


def get_turn_state(current_dir, next_dir):

    directions = [
        Direction.NORTH,
        Direction.EAST,
        Direction.SOUTH,
        Direction.WEST
    ]

    current_index = directions.index(current_dir)
    next_index = directions.index(next_dir)

    difference = (next_index - current_index) % 4

    if difference == 0:
        return RobotState.FOLLOWING

    elif difference == 1:
        # Rotation droite
        return RobotState.TURNING_MINUS_90

    elif difference == 2:
        # Demi-tour
        return RobotState.TURNING_180

    elif difference == 3:
        # Rotation gauche
        return RobotState.TURNING_90


def run_state_machine(
    connection,
    serial,
    mode: Mode = Mode.GRID
):

    APPROACH_DURATION = 0.9

    controller = Control(serial)
    processor = ImageProcessor()

    # ----------------------------------------
    # CONFIGURATION DE LA NAVIGATION
    # ----------------------------------------

    grid = Grid(4)
    nav = Navigation()

    start_pos = (0, 1)
    end_pos = (4, 4)

    robot_dir = Direction.SOUTH

    # Calcul du chemin
    path = nav.generate_path(
        grid,
        start_pos,
        end_pos,
        robot_dir
    )

    if not path:
        print("Impossible de calculer le chemin.")
        controller.stop()
        serial.disconnect()
        return

    # Conversion coordonnées -> directions
    directions = nav.path_to_directions(path)

    print("Chemin :", path)
    print("Directions :", directions)

    nav.display_path(grid, path)

    # ----------------------------------------
    # INITIALISATION MACHINE A ETATS
    # ----------------------------------------

    robot_state = RobotState.FOLLOWING

    action_start_time = connection.get_time()

    # Empêche de détecter plusieurs fois
    # la même intersection
    intersection_armed = True

    # Gestion du demi-tour
    turn_180_phase = 0

    try:

        while connection.step():

            current_time = connection.get_time()

            frame = connection.get_frame()

            cx, is_intersection, roi_w = processor.process_frame(frame)

            # ----------------------------------------
            # FOLLOWING
            # ----------------------------------------

            if robot_state == RobotState.FOLLOWING:

                # Réarmement après avoir quitté
                # complètement l'intersection
                if not is_intersection:
                    intersection_armed = True

                if is_intersection and intersection_armed:

                    intersection_armed = False

                    robot_state = (
                        RobotState.APPROACHING_INTERSECTION
                    )

                    action_start_time = current_time

                    print("Intersection détectée !")

                if cx is not None:

                    error = (
                        (cx - (roi_w / 2))
                        / (roi_w / 2)
                    )

                    controller.aim(error)

                else:

                    controller.forward()

            # ----------------------------------------
            # APPROACHING INTERSECTION
            # ----------------------------------------

            elif robot_state == RobotState.APPROACHING_INTERSECTION:

                controller.forward()

                if (
                    current_time - action_start_time
                    > APPROACH_DURATION
                ):

                    if mode == Mode.HUIT:

                        robot_state = RobotState.FOLLOWING

                    else:

                        robot_state = (
                            RobotState.STOP_ON_INTERSECTION
                        )

                        action_start_time = current_time

                        print(
                            "Arrivée au centre de l'intersection."
                        )

            # ----------------------------------------
            # STOP ON INTERSECTION
            # ----------------------------------------

            elif robot_state == RobotState.STOP_ON_INTERSECTION:

                controller.stop()

                # Si toutes les directions sont parcourues
                if not directions:

                    print("Destination atteinte !")

                    robot_state = RobotState.STOPPED

                else:

                    # Direction suivante
                    next_dir = directions.pop(0)

                    print("Direction suivante :", next_dir)
                    print("Orientation actuelle :", robot_dir)

                    # Calcul de la rotation
                    robot_state = get_turn_state(
                        robot_dir,
                        next_dir
                    )

                    # Mise à jour de l'orientation
                    robot_dir = next_dir

                    action_start_time = current_time

                    # Initialisation demi-tour
                    turn_180_phase = 0

                    print(
                        "Nouvel état :",
                        robot_state
                    )

            # ----------------------------------------
            # TURN LEFT 90°
            # ----------------------------------------

            elif robot_state == RobotState.TURNING_90:

                if controller.turn_90_degrees(
                    left=True,
                    action_start_time=action_start_time
                ):

                    print("Rotation gauche terminée.")

                    robot_state = RobotState.FOLLOWING

            # ----------------------------------------
            # TURN RIGHT 90°
            # ----------------------------------------

            elif robot_state == RobotState.TURNING_MINUS_90:

                if controller.turn_90_degrees(
                    left=False,
                    action_start_time=action_start_time
                ):

                    print("Rotation droite terminée.")

                    robot_state = RobotState.FOLLOWING

            # ----------------------------------------
            # TURN 180°
            # ----------------------------------------

            elif robot_state == RobotState.TURNING_180:

                if controller.turn_90_degrees(
                    left=False,
                    action_start_time=action_start_time
                ):

                    if turn_180_phase == 0:

                        # Première rotation de 90°
                        turn_180_phase = 1

                        action_start_time = current_time

                        print(
                            "Première rotation de 90° terminée."
                        )

                    else:

                        # Deuxième rotation de 90°
                        print("Demi-tour terminé.")

                        robot_state = RobotState.FOLLOWING

                        turn_180_phase = 0

            # ----------------------------------------
            # STOPPED
            # ----------------------------------------

            elif robot_state == RobotState.STOPPED:

                controller.stop()

                print("Robot arrêté.")

                break

    finally:

        controller.stop()

        serial.disconnect()


def main():

    connection, serial = create_environment(SIMULATION)

    run_state_machine(
        connection,
        serial,
        Mode.GRID
    )


if __name__ == "__main__":

    main()
