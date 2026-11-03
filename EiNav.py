from control import Control
from serialpi import Serial
from image_processor import ImageProcessor
from navigation import Grid, Navigation
from robot_state import RobotState
from mode import Mode
from direction import Direction

# Simulation flag: set to True when running in Webots, False on real hardware
SIMULATION = True


def create_environment(simulation: bool):
    """Initializes the appropriate hardware/simulation connection and serial link."""
    if simulation:
        from webots_connection import WebotsConnection
        connection = WebotsConnection()
        serial = Serial(robot=connection.robot)
    else:
        from real_robot_connection import RealRobotConnection
        connection = RealRobotConnection()
        serial = Serial(robot=None)
    return connection, serial


def run_state_machine(connection, serial, mode: Mode = Mode.GRID):
    """Main state machine logic common to both real robot and simulation."""
    APPROACH_DURATION = 0.9

    controller = Control(serial)
    processor = ImageProcessor()

    grid = Grid(4)
    nav = Navigation()
    start_pos = (0, 1)
    end_pos = (3, 3)
    robot_dir = Direction.EAST

    path, path_directions_dict = nav.generate_path(grid, start_pos, end_pos, robot_dir)
    path_directions = [path_directions_dict[coord] for coord in path[:-1]]

    robot_state = RobotState.FOLLOWING
    action_start_time = 0.0

    try:
        while connection.step():
            current_time = connection.get_time()
            frame = connection.get_frame()

            cx, is_intersection, roi_w = processor.process_frame(frame)

            if robot_state == RobotState.FOLLOWING:
                if is_intersection:
                    robot_state = RobotState.APPROACHING_INTERSECTION
                    action_start_time = current_time
                    print("Intersection détectée ! Avancée vers le centre.")

                if cx is not None:
                    error = (cx - (roi_w / 2))/(roi_w / 2)
                    controller.aim(error)
                else:
                    controller.forward()

            elif robot_state == RobotState.APPROACHING_INTERSECTION:
                controller.forward()
                if current_time - action_start_time > APPROACH_DURATION:
                    if mode == Mode.HUIT:
                        robot_state = RobotState.FOLLOWING
                    else:
                        robot_state = RobotState.STOP_ON_INTERSECTION
                        action_start_time = current_time
                        print("Arrivée au centre de l'intersection. Arrêt momentané.")

            elif robot_state == RobotState.STOP_ON_INTERSECTION:
                controller.stop()
                if len(path_directions) == 1:
                    print("Arret du robot")
                    robot_state = RobotState.STOPPED
                else:
                    next_dir = path_directions.pop(0) if path_directions else None
                    action_start_time = current_time

                    if next_dir == '>':
                        robot_state = RobotState.TURNING_90
                    elif next_dir == '<':
                        robot_state = RobotState.TURNING_MINUS_90
                    elif next_dir == '^':
                        robot_state = RobotState.FOLLOWING
                    elif next_dir == 'v':
                        robot_state = RobotState.TURNING_180
                    else:
                        robot_state = RobotState.FOLLOWING

            elif robot_state == RobotState.TURNING_90:
                if controller.turn_90_degrees(left=True, action_start_time=action_start_time):
                    robot_state = RobotState.FOLLOWING

            elif robot_state == RobotState.TURNING_MINUS_90:
                if controller.turn_90_degrees(left=False, action_start_time=action_start_time):
                    robot_state = RobotState.FOLLOWING

            elif robot_state == RobotState.TURNING_180:
                if controller.turn_90_degrees(left=True, action_start_time=action_start_time):
                    robot_state = RobotState.FOLLOWING

    finally:
        controller.stop()
        serial.disconnect()


def main():
    connection, serial = create_environment(SIMULATION)
    run_state_machine(connection, serial)


if __name__ == "__main__":
    main()
