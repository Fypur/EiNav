# General imports
from robot_controller import turn_90_degrees
import numpy as np
import cv2

# Specific imports for Webots
from controller import Robot
from controller import Camera

#####################
# Robot setup and control
#####################

speed_constant = 10  # Vitesse de base du robot

#####################
# Webots setup
#####################

TIME_STEP = 64
robot = Robot()

# Enable camera
camera = robot.getDevice("PiCam")
camera.enable(TIME_STEP)

cv2.namedWindow("Robot - Vue Camera & Suivi", cv2.WINDOW_NORMAL)
# cv2.namedWindow("Traitement d'image - 4 Etapes", cv2.WINDOW_NORMAL)

# Variables pour la machine à états de l'intersection et de la rotation
robot_state = "FOLLOWING"  # États : "FOLLOWING", "APPROACHING_INTERSECTION", "TURNING_90"
action_start_time = 0
APPROACH_DURATION = 0.9    # Temps pour avancer jusqu'au cœur de l'intersection (à ajuster si besoin)
TURN_DURATION = 1.0        # Durée de la rotation de 90°
intersection_counter = 0

#####################
# Main loop
#####################

while robot.step(TIME_STEP) != -1:
    current_time = robot.getTime()
    
    #####################
    # Image processing
    #####################

    # Read image from webots camera (BGRA)
    cameraData = camera.getImage()
    image = np.frombuffer(cameraData, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )

    # Conversion en BGR standard
    image_bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    h, w = image_bgr.shape[:2]

    # Définition de la Zone d'Intérêt tiers inférieur de l'image
    roi_top = int(h * 0.6)
    roi = image_bgr[roi_top:h, 0:w]
    roi_h, roi_w = roi.shape[:2]

    # Conversion en niveaux de gris et lissage
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur_roi = cv2.GaussianBlur(gray_roi, (9, 9), 0)
    step1_img = cv2.cvtColor(blur_roi, cv2.COLOR_GRAY2BGR)
    cv2.putText(step1_img, "1. Flou Lisse", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # threshold
    _, thresh_roi = cv2.threshold(blur_roi, 140, 255, cv2.THRESH_BINARY)
    step2_img = cv2.cvtColor(thresh_roi, cv2.COLOR_GRAY2BGR)
    cv2.putText(step2_img, "2. Seuil (Threshold)", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # --- ETAPE 3 : Morphologie avancée ---
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    morphed = cv2.morphologyEx(thresh_roi, cv2.MORPH_CLOSE, kernel)
    morphed = cv2.morphologyEx(morphed, cv2.MORPH_OPEN, kernel)
    
    step3_img = cv2.cvtColor(morphed, cv2.COLOR_GRAY2BGR)
    cv2.putText(step3_img, "3. Morphologie Lisse", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Reco
    contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    full_view_img = image_bgr.copy()
    step4_img = roi.copy()
    cx, cy = None, None
    is_intersection = False

    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)
        contour_area = cv2.contourArea(max_contour)
        
        if contour_area > 50:
            # Condition robuste d'intersection par les bords de la ROI

            # a modifier en focntion de si on s'attend à un croisement à 4, un T ou un L
            touches_top = np.any(morphed[0, :] > 0)
            touches_bottom = np.any(morphed[roi_h - 1, :] > 0)
            touches_left = np.any(morphed[:, 0] > 0)
            touches_right = np.any(morphed[:, roi_w - 1] > 0)

            if touches_top and touches_bottom and (touches_left or touches_right):
                is_intersection = True

            epsilon = 0.02 * cv2.arcLength(max_contour, True)
            smooth_contour = cv2.approxPolyDP(max_contour, epsilon, True)

            # Dessin de la trajectoire sur la ROI (étape 4)
            cv2.drawContours(step4_img, [smooth_contour], -1, (150, 255, 150), 6)
            cv2.drawContours(step4_img, [smooth_contour], -1, (0, 255, 0), 2)

            # Traduction des coordonnées pour l'image entière
            full_contour = smooth_contour.copy()
            full_contour[:, :, 1] += roi_top

            cv2.drawContours(full_view_img, [full_contour], -1, (150, 255, 150), 6)
            cv2.drawContours(full_view_img, [full_contour], -1, (0, 255, 0), 2)

            M = cv2.moments(max_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                cv2.circle(step4_img, (cx, cy), 6, (0, 0, 255), -1)
                cv2.circle(full_view_img, (cx, cy + roi_top), 6, (0, 0, 255), -1)


    cv2.rectangle(full_view_img, (0, roi_top), (w, h), (0, 255, 255), 2)

    status_text = robot_state if robot_state != "FOLLOWING" else ("INTERSECTION !" if is_intersection else "Suivi de ligne")
    text_color = (0, 0, 255) if is_intersection or robot_state != "FOLLOWING" else (0, 255, 0)
    
    cv2.putText(step4_img, status_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
    cv2.putText(full_view_img, f"Etat: {status_text}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

    
    if robot_state == "FOLLOWING":
        if is_intersection:
            robot_state = "APPROACHING_INTERSECTION"
            action_start_time = current_time
            intersection_counter += 1
            
            filename = f"intersection_debug_{intersection_counter}.jpg"
            cv2.imwrite(filename, np.vstack([np.hstack([step1_img, step2_img]), np.hstack([step3_img, step4_img])]))
            print(f"Intersection détectée ! Avancée vers le centre du coin. Image : {filename}")

        # Asservissement du suivi de ligne à modif potentiellement
        if cx is not None:
            error = cx - (roi_w / 2)
            Kp = 0.035
            base_speed = speed_constant
            correction = error * Kp

            left_speed = base_speed + correction
            right_speed = base_speed - correction
        else:
            left_speed = speed_constant * 0.4
            right_speed = speed_constant * 0.4

    elif robot_state == "APPROACHING_INTERSECTION":
        # Le robot continue d'avancer tout droit un court instant pour se positionner au centre 
        left_speed = speed_constant * 0.8
        right_speed = speed_constant * 0.8

        if current_time - action_start_time > APPROACH_DURATION:
            robot_state = "STOP_ON_INTERSECTION"
            action_start_time = current_time
            print("Arrivée au centre de l'intersection. Arrêt momentané.")

    elif robot_state == "STOP_ON_INTERSECTION":
        left_speed = 0
        right_speed = 0
        robot_state = "TURNING_90"
        action_start_time = current_time
        print("Arrêt complet. Début de la rotation de 90°.")

    elif robot_state == "TURNING_90":
        robot_state = turn_90_degrees(robot, left=True, action_start_time=action_start_time)
        
    set_motors_velocities(left_speed, right_speed)

    top_row = np.hstack([step1_img, step2_img])
    bottom_row = np.hstack([step3_img, step4_img])
    grid_display = np.vstack([top_row, bottom_row])

    # cv2.imshow("Robot - Vue Camera & Suivi", full_view_img)
    cv2.imshow("Traitement d'image - 4 Etapes", grid_display)
    cv2.waitKey(3)
