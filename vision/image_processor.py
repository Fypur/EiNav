from __future__ import annotations

from typing import NamedTuple, Optional

import cv2
import numpy as np

from vision import utils


class LineDetection(NamedTuple):
    cx: Optional[int]  # abscisse du centre de la ligne (None : pas de ligne)
    is_intersection: bool
    roi_w: int
    roi_top: int  # ordonnée du haut de la zone d'intérêt dans l'image
    mask: np.ndarray  # image binaire de la zone d'intérêt


def _widest_run(mask: np.ndarray) -> int:
    """Plus grand nombre de pixels blancs consécutifs sur une même ligne de l'image binaire."""
    widest = 0
    for row in mask > 0:
        edges = np.flatnonzero(np.diff(np.concatenate(([False], row, [False])).astype(np.int8)))
        if edges.size:
            widest = max(widest, int((edges[1::2] - edges[::2]).max()))
    return widest


class ImageProcessor:
    def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
        """Applique le pipeline de traitement sur l'image BGR et retourne la détection."""
        h, w = image_bgr.shape[:2]

        # Définition de la Zone d'Intérêt (tiers inférieur)
        roi_top = int(h * utils.ROI_TOP_RATIO)
        roi = image_bgr[roi_top:h, 0:w]
        roi_h, roi_w = roi.shape[:2]

        # --- Etape 1 : Flou ---
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur_roi = cv2.GaussianBlur(gray_roi, utils.BLUR_KERNEL, 0)

        # --- Etape 2 : Seuil ---
        _, thresh_roi = cv2.threshold(blur_roi, utils.BINARY_THRESHOLD, 255, cv2.THRESH_BINARY)

        # --- Etape 3 : Morphologie ---
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, utils.MORPH_KERNEL)
        morphed = cv2.morphologyEx(thresh_roi, cv2.MORPH_CLOSE, kernel)
        morphed = cv2.morphologyEx(morphed, cv2.MORPH_OPEN, kernel)

        # --- Etape 4 : Reconnaissance des contours et intersections ---
        contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cx: Optional[int] = None
        is_intersection = False

        if len(contours) > 0:
            max_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(max_contour) > utils.MIN_LINE_AREA:
                touches_top = np.any(morphed[0, :] > 0)
                touches_bottom = np.any(morphed[roi_h - 1, :] > 0)
                touches_left = np.any(morphed[:, 0] > 0)
                touches_right = np.any(morphed[:, roi_w - 1] > 0)

                # Une ligne en biais qui passe par un coin touche aussi le haut, le bas et un côté :
                # seul un croisement a en plus une large bande horizontale.
                if touches_top and touches_bottom and (touches_left or touches_right):
                    is_intersection = _widest_run(morphed) >= utils.INTERSECTION_MIN_WIDTH_RATIO * roi_w

                M = cv2.moments(max_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])

        return LineDetection(cx, bool(is_intersection), roi_w, roi_top, morphed)

    def annotate(self, image_bgr: np.ndarray, detection: LineDetection) -> np.ndarray:
        """Image traitée pour le tableau de bord : zone d'intérêt, ligne détectée et son centre."""
        annotated = image_bgr.copy()
        h, w = annotated.shape[:2]
        roi = annotated[detection.roi_top:h, 0:w]
        roi[detection.mask > 0] = utils.LINE_COLOR
        cv2.rectangle(annotated, (0, detection.roi_top), (w - 1, h - 1),
                      utils.INTERSECTION_COLOR if detection.is_intersection else utils.ROI_COLOR, 1)
        if detection.cx is not None:
            cv2.line(annotated, (detection.cx, detection.roi_top), (detection.cx, h - 1), utils.TARGET_COLOR, 1)
        return annotated
