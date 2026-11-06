from __future__ import annotations

from typing import NamedTuple, Optional, Tuple

import cv2
import numpy as np

from vision import utils


class LineDetection(NamedTuple):
    cx: Optional[int]  # abscisse du centre de la ligne dans l'image (None si pas de ligne)
    is_intersection: bool
    image_w: int  # largeur de l'image, le suivi de ligne vise son milieu
    roi_top: int  # ordonnée du haut de la zone d'intérêt dans l'image
    mask: np.ndarray  # image binaire de la zone d'intérêt
    roi_left: int = 0  # abscisse du bord gauche de la zone d'intérêt dans l'image


def _roi_span(size: int, start_ratio: float, end_ratio: float) -> Tuple[int, int]:
    """Bornes en pixels de la zone sur un axe. Garde une taille min même si les réglages se croisent."""
    start = int(size * start_ratio)
    return start, max(int(size * end_ratio), start + utils.ROI_MIN_PIXELS)


def _widest_run(mask: np.ndarray) -> int:
    """Plus grande suite de pixels blancs sur une même ligne de l'image binaire."""
    widest = 0
    for row in mask > 0:
        edges = np.flatnonzero(np.diff(np.concatenate(([False], row, [False])).astype(np.int8)))
        if edges.size:
            widest = max(widest, int((edges[1::2] - edges[::2]).max()))
    return widest


class ImageProcessor:
    def process_frame(self, image_bgr: np.ndarray) -> LineDetection:
        """Traite l'image BGR et renvoie la détection."""
        h, w = image_bgr.shape[:2]

        # Zone d'intérêt : le rectangle réglable depuis l'interface
        roi_top, roi_bottom = _roi_span(h, utils.ROI_TOP_RATIO, utils.ROI_BOTTOM_RATIO)
        roi_left, roi_right = _roi_span(w, utils.ROI_LEFT_RATIO, utils.ROI_RIGHT_RATIO)
        roi = image_bgr[roi_top:roi_bottom, roi_left:roi_right]
        roi_h, roi_w = roi.shape[:2]

        # Etape 1 : flou
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur_roi = cv2.GaussianBlur(gray_roi, utils.BLUR_KERNEL, 0)

        # Etape 2 : seuil
        _, thresh_roi = cv2.threshold(blur_roi, utils.BINARY_THRESHOLD, 255, cv2.THRESH_BINARY)

        # Etape 3 : morphologie
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, utils.MORPH_KERNEL)
        morphed = cv2.morphologyEx(thresh_roi, cv2.MORPH_CLOSE, kernel)
        morphed = cv2.morphologyEx(morphed, cv2.MORPH_OPEN, kernel)

        # Etape 4 : contours et intersections
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

                # Une ligne en biais qui passe par un coin touche aussi le haut, le bas et un côté,
                # donc on vérifie en plus la large bande horizontale du croisement
                if touches_top and touches_bottom and (touches_left or touches_right):
                    # Par rapport à l'image et pas à la zone : dans une zone étroite, une simple ligne serait assez large
                    is_intersection = _widest_run(morphed) >= utils.INTERSECTION_MIN_WIDTH_RATIO * w

                M = cv2.moments(max_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00']) + roi_left

        return LineDetection(cx, bool(is_intersection), w, roi_top, morphed, roi_left)

    def annotate(self, image_bgr: np.ndarray, detection: LineDetection) -> np.ndarray:
        """Image pour l'interface, avec la zone d'intérêt, la ligne et son centre."""
        annotated = image_bgr.copy()
        roi_h, roi_w = detection.mask.shape[:2]
        bottom, right = detection.roi_top + roi_h, detection.roi_left + roi_w
        roi = annotated[detection.roi_top:bottom, detection.roi_left:right]
        roi[detection.mask > 0] = utils.LINE_COLOR
        cv2.rectangle(annotated, (detection.roi_left, detection.roi_top), (right - 1, bottom - 1),
                      utils.INTERSECTION_COLOR if detection.is_intersection else utils.ROI_COLOR, 1)
        if detection.cx is not None:
            cv2.line(annotated, (detection.cx, detection.roi_top), (detection.cx, bottom - 1), utils.TARGET_COLOR, 1)
        return annotated
