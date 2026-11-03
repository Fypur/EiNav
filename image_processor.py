import cv2
import numpy as np

class ImageProcessor:
    def __init__(self, camera, use_webots = True):
        self.camera = camera
        self.use_webots = use_webots

    def get_image(self) :
        if self.use_webots :
            cameraData = self.camera.getImage()
            image = np.frombuffer(cameraData, np.uint8).reshape(
                (self.camera.getHeight(), self.camera.getWidth(), 4)
            )
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        else:
            return self.camera.read()

    def process_frame(self):
        """
        Lit l'image de la caméra, applique le pipeline de traitement 
        et retourne les informations utiles (centre de la ligne cx, indicateur d'intersection, images de debug).
        """
        image_bgr = self.get_image()
        h, w = image_bgr.shape[:2]

        # Définition de la Zone d'Intérêt (tiers inférieur)
        roi_top = int(h * 0.6)
        roi = image_bgr[roi_top:h, 0:w]
        roi_h, roi_w = roi.shape[:2]

        # --- Etape 1 : Flou ---
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur_roi = cv2.GaussianBlur(gray_roi, (9, 9), 0)
        step1_img = cv2.cvtColor(blur_roi, cv2.COLOR_GRAY2BGR)
        cv2.putText(step1_img, "1. Flou Lisse", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # --- Etape 2 : Seuil ---
        _, thresh_roi = cv2.threshold(blur_roi, 140, 255, cv2.THRESH_BINARY)
        step2_img = cv2.cvtColor(thresh_roi, cv2.COLOR_GRAY2BGR)
        cv2.putText(step2_img, "2. Seuil (Threshold)", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # --- Etape 3 : Morphologie ---
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        morphed = cv2.morphologyEx(thresh_roi, cv2.MORPH_CLOSE, kernel)
        morphed = cv2.morphologyEx(morphed, cv2.MORPH_OPEN, kernel)
        
        step3_img = cv2.cvtColor(morphed, cv2.COLOR_GRAY2BGR)
        cv2.putText(step3_img, "3. Morphologie Lisse", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # --- Etape 4 : Reconnaissance des contours et intersections ---
        contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        step4_img = roi.copy()
        cx = None
        is_intersection = False

        if len(contours) > 0:
            max_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(max_contour) > 50:
                # Détection d'intersection (touche les bords de la ROI)
                touches_top = np.any(morphed[0, :] > 0)
                touches_bottom = np.any(morphed[roi_h - 1, :] > 0)
                touches_left = np.any(morphed[:, 0] > 0)
                touches_right = np.any(morphed[:, roi_w - 1] > 0)

                if touches_top and touches_bottom and (touches_left or touches_right):
                    is_intersection = True

                epsilon = 0.02 * cv2.arcLength(max_contour, True)
                smooth_contour = cv2.approxPolyDP(max_contour, epsilon, True)
                cv2.drawContours(step4_img, [smooth_contour], -1, (0, 255, 0), 2)

                # Calcul du centre de masse (cx)
                M = cv2.moments(max_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cv2.circle(step4_img, (cx, int(M['m01'] / M['m00'])), 6, (0, 0, 255), -1)

        debug_images = {
            "step1": step1_img,
            "step2": step2_img,
            "step3": step3_img,
            "step4": step4_img
        }

        return cx, is_intersection, roi_w, debug_images