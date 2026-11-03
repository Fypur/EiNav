import cv2
import numpy as np


class ImageProcessor:
    def process_frame(self, image_bgr: np.ndarray):
        """
        Applique le pipeline de traitement sur l'image BGR
        et retourne (cx, is_intersection, roi_w).
        """
        h, w = image_bgr.shape[:2]

        # Définition de la Zone d'Intérêt (tiers inférieur)
        roi_top = int(h * 0.6)
        roi = image_bgr[roi_top:h, 0:w]
        roi_h, roi_w = roi.shape[:2]

        # --- Etape 1 : Flou ---
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur_roi = cv2.GaussianBlur(gray_roi, (9, 9), 0)

        # --- Etape 2 : Seuil ---
        _, thresh_roi = cv2.threshold(blur_roi, 140, 255, cv2.THRESH_BINARY)

        # --- Etape 3 : Morphologie ---
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        morphed = cv2.morphologyEx(thresh_roi, cv2.MORPH_CLOSE, kernel)
        morphed = cv2.morphologyEx(morphed, cv2.MORPH_OPEN, kernel)

        # --- Etape 4 : Reconnaissance des contours et intersections ---
        contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cx = None
        is_intersection = False

        if len(contours) > 0:
            max_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(max_contour) > 50:
                touches_top = np.any(morphed[0, :] > 0)
                touches_bottom = np.any(morphed[roi_h - 1, :] > 0)
                touches_left = np.any(morphed[:, 0] > 0)
                touches_right = np.any(morphed[:, roi_w - 1] > 0)

                if touches_top and touches_bottom and (touches_left or touches_right):
                    is_intersection = True

                M = cv2.moments(max_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])

        return cx, is_intersection, roi_w
