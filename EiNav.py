from controller import Robot
import cv2
import numpy as np
from robot_controller import RobotController
from image_processor import ImageProcessor
from navigation import Grid, Navigation
from robot_state import RobotState
from mode import Mode
from direction import Direction

def main():
    TIME_STEP = 64
    robot = Robot()

    mode = Mode.HUIT

    controller = RobotController(robot)
    
    camera = robot.getDevice("PiCam")
    camera.enable(TIME_STEP)
    processor = ImageProcessor(camera)

    cv2.namedWindow("Traitement d'image - 4 Etapes", cv2.WINDOW_NORMAL)

    # Variables de la machine à états
    robot_state = RobotState.FOLLOWING  
    action_start_time = 0
    APPROACH_DURATION = 0.9    
    intersection_counter = 0

    grid = Grid(4)
    nav = Navigation()
    
    start_pos = (0, 0)
    end_pos = (2, 2)
    robot_dir = Direction.EAST
    
    path_directions = nav.generate_path(grid, start_pos, end_pos, robot_dir)[1]
    while robot.step(TIME_STEP) != -1:
        current_time = robot.getTime()
        
        # --- 1. Traitement d'image via la classe dédiée ---
        cx, is_intersection, roi_w, debug_images = processor.process_frame()

        # Affichage du statut sur l'étape 4
        status_text = robot_state.name if robot_state != RobotState.FOLLOWING else ("INTERSECTION !" if is_intersection else "Suivi de ligne")
        text_color = (0, 0, 255) if is_intersection or robot_state != RobotState.FOLLOWING else (0, 255, 0)
        cv2.putText(debug_images["step4"], status_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)

        # --- 2. Machine à états ---
        if robot_state == RobotState.FOLLOWING:
            if is_intersection:
                robot_state = RobotState.APPROACHING_INTERSECTION
                action_start_time = current_time
                intersection_counter += 1
                
                # Sauvegarde de l'image de debug
                filename = f"intersection_debug_{intersection_counter}.jpg"
                grid_debug = np.vstack([
                    np.hstack([debug_images["step1"], debug_images["step2"]]), 
                    np.hstack([debug_images["step3"], debug_images["step4"]])
                ])
                cv2.imwrite(filename, grid_debug)
                print(f"Intersection détectée ! Avancée vers le centre. Image : {filename}")

            # Asservissement (Suivi de ligne)
            if cx is not None:
                error = cx - (roi_w / 2)
                left_speed, right_speed = controller.apply_pid(error)
            else:
                left_speed = controller.speed_constant * 0.4
                right_speed = controller.speed_constant * 0.4
            
            controller.set_motors_velocities(left_speed, right_speed)

        elif robot_state == RobotState.APPROACHING_INTERSECTION:
            controller.set_motors_velocities(controller.speed_constant * 0.8, controller.speed_constant * 0.8)
            if current_time - action_start_time > APPROACH_DURATION:
                if mode == Mode.HUIT:
                    robot_state = RobotState.FOLLOWING
                else:
                    robot_state = RobotState.STOP_ON_INTERSECTION
                    action_start_time = current_time
                    print("Arrivée au centre de l'intersection. Arrêt momentané.")

        elif robot_state == RobotState.STOP_ON_INTERSECTION:
            
            next_dir = path_directions[0] if path_directions else None
            if next_dir == '>':
                controller.set_motors_velocities(0, 0)
                robot_state = RobotState.TURNING_90
            elif next_dir == '<':
                controller.set_motors_velocities(0, 0)
                robot_state = RobotState.TURNING_MINUS_90
            elif next_dir == '^':
                robot_state = RobotState.FOLLOWING
            elif next_dir == 'v':
                controller.set_motors_velocities(0, 0)
                robot_state = RobotState.TURNING_180

                path_directions.pop(0)
            action_start_time = current_time

        elif robot_state == RobotState.TURNING_90:
            robot_state = controller.turn_90_degrees(left=True, action_start_time=action_start_time)

        elif robot_state == RobotState.TURNING_MINUS_90:
            robot_state = controller.turn_90_degrees(left=False, action_start_time=action_start_time)

        elif robot_state == RobotState.TURNING_180:
            robot_state = controller.turn_90_degrees(left=True, action_start_time=action_start_time)
            robot_state = controller.turn_90_degrees(left=True, action_start_time=action_start_time)

        # --- 3. Affichage global de débogage ---
        top_row = np.hstack([debug_images["step1"], debug_images["step2"]])
        bottom_row = np.hstack([debug_images["step3"], debug_images["step4"]])
        grid_display = np.vstack([top_row, bottom_row])
        
        cv2.imshow("Traitement d'image - 4 Etapes", grid_display)
        cv2.waitKey(3)

if __name__ == "__main__":
    main()
