import cv2
import imutils
import numpy as np


class DetectCentre:
    """
    A class to detect solid circles and their centres.
    ...
    Attributes:
        lower : tuple
            collection of three integer values for RGB lower bound of a color (used to detect masks in image)
        upper : tuple
            collection of three integer values for RGB upper bound of a color (used to detect masks in image)
        resize : int
            resize value for the frame
        min_radius : int
            minimum radius used to filter circles
    Methods:
        __call__(frame: np.ndarray):
            Accepts a frame and returns a list of the centres and radius of all circles detected
    """

    def __init__(self, lower=(29, 86, 6), upper=(64, 255, 255), resize=600, min_radius=1):
        """Initialize the DetectCentre class
        Parameters:
            lower (tuple): RGB values for the lower bound of color spectrum to detect mask
            upper (tuple): RGB values for the upper bound of color spectrum to detect mask
            resize (int): Image size to resize to
            min_radius (int): minimum radius to filter circles
        """
        self._lower = lower
        self._upper = upper
        self._resize = resize
        self._min_radius = min_radius
        self._all_detections = []

    def __repr__(self):
        """Returns the lower and upper bounds of a mask used to detect solid circles.
        Returns:
            str: information string
        """
        return "Bounds of masK: " + str(self._lower) + str(self._upper)

    def __call__(self, frame: np.ndarray, **kwargs) -> list:
        """Accept an image and detect circles in it.
        Parameters:
            frame (np.ndarray): Image in np.ndarray format
        Returns:
            A list consisting information about the centres and radius of all circles detected. The list format
            is [[x1, y1, r1][x2, y2, r2]] where x1, y1 are centres of circle 1, r1 is radius of circle 1, x2, y2 are
            centres of circle 2, r2 is radius of circle 2.
        """
        circles = []
        img = frame.copy()
        img = imutils.resize(img, width=self._resize)
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        for contour in contours:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            if radius > 1:
                circles.append([x, y, radius])
                self._all_detections.append([x, y, radius])

        return circles
