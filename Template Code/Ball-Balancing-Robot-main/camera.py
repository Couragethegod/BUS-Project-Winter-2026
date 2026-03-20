# Standard libraries
import time
import cv2
from picamera2 import Picamera2, Preview
import numpy as np

# Used for smoothing detected positions over time
from collections import deque

import threading


class Camera:
    # Native resolution example: 1640x1232
    def __init__(self, resolution=(1640, 1232), format="RGB888"):
        """
        Camera Initialization
        
        resolution: tuple (width, height)
            Higher → more detail, slower processing
        
        format: pixel format (RGB888 works well with OpenCV)
        """

        # Initialize Pi camera
        self.picam2 = Picamera2()

        # Configure camera preview
        # FrameDurationLimits controls FPS (~120 FPS here)
        config = self.picam2.create_preview_configuration(
            main={"size": resolution, "format": format},
            controls={"FrameDurationLimits": (8333, 8333)}
        )

        self.picam2.configure(config)

        # ------------------- COLOR FILTERING -------------------
        # HSV range for detecting YELLOW objects
        # You can adjust these for different lighting conditions
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])


        # ------------------- POSITION SMOOTHING -------------------
        # Stores last N detected positions
        # Helps reduce noise and jitter
        self.queue = deque(maxlen=16)

        # Initial fallback position (center-ish)
        self.queue.append((100, 75))  

        # Start camera
        self.picam2.start()    


    def take_picture(self):
        """
        Capture image and resize for faster processing
        
        Returns:
            resized frame (width = 200 px)
        """

        image = self.picam2.capture_array()

        # Resize image to fixed width (200 px)
        # This greatly improves performance
        scale = 200.0 / image.shape[1]
        frame_resized = cv2.resize(
            image,
            (200, int(image.shape[0] * scale))
        )

        return frame_resized


    def display(self, image, window_name="Camera Output"):
        """
        Display raw image
        """
        cv2.imshow(window_name, image)
        cv2.waitKey(1) 


    def display_draw(self, image, center, window_name="Tracked Output"):
        """
        Draw a crosshair on detected object
        
        center: (x, y) position
        """

        x, y = center

        # Draw horizontal line
        cv2.line(image, (x - 10, y), (x + 10, y), (0, 0, 255), 2)

        # Draw vertical line
        cv2.line(image, (x, y - 10), (x, y + 10), (0, 0, 255), 2)

        cv2.imshow(window_name, image)
        cv2.waitKey(1) 


    def terminate(self):
        """
        Properly close camera and windows
        """
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()


    def coordinate(self, image):
        """
        Detects a DARK (black) circular object in the image
        
        Returns:
            (x, y) position of detected object
        """

        prev_time = time.time()  # (currently unused)

        # ------------------- PREPROCESSING -------------------

        # Apply Gaussian blur to reduce noise
        frame_blurred = cv2.GaussianBlur(image, (3, 3), 0)
        
        # Convert to HSV (better for color filtering)
        frame_hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)

        mask_combined = cv2.inRange(frame_hsv, self.lower_yellow, self.upper_yellow)

        # ------------------- MORPHOLOGICAL CLEANUP -------------------

        kernel = np.ones((5, 5), np.uint8)
        mask_dilated = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, kernel)
        mask_dilated = cv2.morphologyEx(mask_dilated, cv2.MORPH_CLOSE, kernel)

        # ------------------- CONTOUR DETECTION -------------------

        valid_detections = []

        contours, _ = cv2.findContours(
            mask_dilated.copy(),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            # Fit a circle around contour
            (x, y), radius = cv2.minEnclosingCircle(contour)
            radius = int(radius)

            # ------------------- SIZE FILTER -------------------
            # Ignore objects too small or too large
            if radius < 5 or radius > 100:
                continue

            # ------------------- CIRCULARITY CHECK -------------------
            # Formula: 4π * (Area / Perimeter²)
            # Circle ≈ 1.0, irregular shapes < 1
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            if perimeter == 0:
                continue

            circularity = (4 * np.pi * area) / (perimeter ** 2)

            # Reject non-circular shapes
            if circularity < 0.6:
                continue

            # ------------------- CENTER CALCULATION -------------------

            x, y, w, h = cv2.boundingRect(contour)

            # Store (area, center)
            valid_detections.append(
                (area, (int(x + w / 2), int(y + h / 2)))
            )

        
        # ------------------- SELECT BEST DETECTION -------------------

        if valid_detections:
            # Choose largest object
            best_center = max(valid_detections, key=lambda item: item[0])[1]

            # Add to smoothing queue
            self.queue.append(best_center)

        else:
            # If no detection, reuse last known position
            self.queue.append(self.queue[-1])

        # Return smoothed/latest position
        return self.queue[-1]
        

# ------------------- TEST SCRIPT -------------------

if __name__ == "__main__":
    cam = Camera()

    try:
        while True:
            # Capture frame
            img = cam.take_picture()

            # Detect object
            c = cam.coordinate(img)

            # Display result
            cam.display_draw(img, c)

            print(c)
            
            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Cleanup resources
        cam.terminate()
