# Standard libraries
import time
import cv2
from picamera2 import Picamera2
import numpy as np

# Used for smoothing detected positions over time
from collections import deque


class Camera:
    def __init__(self, resolution=(640, 480), format="RGB888", display_width=None):
        """
        Camera Initialization

        resolution: (width, height)
            Recommended: (640, 480) for good balance

        display_width:
            If set, image will be resized for processing/display
            If None → no resizing (uses full resolution)
        """

        self.picam2 = Picamera2()

        # === FPS CONTROL ===
        # 16666 µs → ~60 FPS
        config = self.picam2.create_preview_configuration(
            main={"size": resolution, "format": format},
            controls={"FrameDurationLimits": (16666, 16666)}
        )

        self.picam2.configure(config)

        # Optional downscaling (for speed tuning)
        self.display_width = display_width

        # ================= COLOR FILTERING =================
        # Default: BLACK ball detection
        self.lower_color = np.array([0, 0, 0])
        self.upper_color = np.array([180, 255, 50])

        # ================= SMOOTHING =================
        self.queue = deque(maxlen=10)  # smaller buffer = faster response
        self.queue.append((100, 75))

        # Start camera
        self.picam2.start()


    def take_picture(self):
        """
        Capture image (optionally resized)
        """

        image = self.picam2.capture_array()

        # Optional resizing (ONLY if needed)
        if self.display_width is not None:
            scale = self.display_width / image.shape[1]
            image = cv2.resize(image, (self.display_width, int(image.shape[0] * scale)))

        return image


    def display(self, image, window_name="Camera Output"):
        cv2.imshow(window_name, image)
        cv2.waitKey(1)


    def display_draw(self, image, center, window_name="Tracked Output"):
        x, y = center

        # Crosshair
        cv2.line(image, (x - 10, y), (x + 10, y), (0, 0, 255), 2)
        cv2.line(image, (x, y - 10), (x, y + 10), (0, 0, 255), 2)

        cv2.imshow(window_name, image)
        cv2.waitKey(1)


    def terminate(self):
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()


    def coordinate(self, image):
        """
        Detect ball position using color filtering
        """

        # ================= PREPROCESSING =================

        # Stronger blur → better noise rejection
        frame_blurred = cv2.GaussianBlur(image, (7, 7), 0)

        # Convert to HSV
        frame_hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)

        # ================= MASK =================

        # Color-only detection (removed grayscale interference)
        mask = cv2.inRange(frame_hsv, self.lower_color, self.upper_color)

        # ================= MORPHOLOGY =================

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ================= CONTOURS =================

        contours, _ = cv2.findContours(
            mask.copy(),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        valid_detections = []

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < 50:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue

            circularity = (4 * np.pi * area) / (perimeter ** 2)

            # Stricter circularity → fewer false positives
            if circularity < 0.75:
                continue

            # More accurate center (centroid instead of bounding box)
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            valid_detections.append((area, (cx, cy)))

        # ================= SELECT BEST =================

        if valid_detections:
            best_center = max(valid_detections, key=lambda item: item[0])[1]
            self.queue.append(best_center)
        else:
            self.queue.append(self.queue[-1])

        # ================= SMOOTHING =================

        avg_x = int(np.mean([p[0] for p in self.queue]))
        avg_y = int(np.mean([p[1] for p in self.queue]))

        return (avg_x, avg_y)


# ================= TEST =================

if __name__ == "__main__":
    # Recommended config:
    # resolution = (640, 480)
    # display_width = None (full accuracy)

    cam = Camera(resolution=(640, 480), display_width=None)

    try:
        while True:
            img = cam.take_picture()
            c = cam.coordinate(img)

            cam.display_draw(img, c)

            print(c)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cam.terminate()