# Import required libraries
from picamera2 import Picamera2   # Library to control Raspberry Pi camera
import cv2                        # OpenCV for image processing
import numpy as np                # Numerical operations (arrays, etc.)
import threading                 # For thread safety when using camera in multiple threads

# Create a lock (not currently used in this code, but useful if multiple threads access the camera)
lock = threading.Lock()


class Camera:
    def __init__(self):
        """
        Constructor: Initializes the camera and sets up all parameters.
        This runs automatically when you create a Camera object.
        """
        
        # Initialize the Pi Camera
        self.picam2 = Picamera2()

        # Set resolution (height and width of captured image)
        # You can change these values depending on your needs:
        # - Lower values = faster processing, lower quality
        # - Higher values = better quality, slower processing
        self.height = 480
        self.width = 480
        
        # Create camera configuration
        # main:
        #   format: Pixel format (XRGB8888 is common for OpenCV compatibility)
        #   size: Resolution (width, height)
        #
        # controls:
        #   FrameDurationLimits: Controls FPS (8333 µs ≈ 120 FPS)
        #       Increase value → lower FPS
        #       Decrease value → higher FPS
        #
        #   ExposureTime: Camera exposure in microseconds
        #       Increase → brighter image
        #       Decrease → darker image
        self.config = self.picam2.create_video_configuration(
            main={"format": 'XRGB8888', "size": (self.height, self.width)},
            controls={
                "FrameDurationLimits": (8333, 8333),
                "ExposureTime": 8000
            }
        )

        # Apply the configuration to the camera
        self.picam2.configure(self.config)

        # Define HSV color range for detecting a pink object
        # HSV = Hue, Saturation, Value (better for color detection than RGB)
        #
        # These values are VERY important and should be tuned depending on lighting:
        # lower_pink = minimum HSV value
        # upper_pink = maximum HSV value
        #
        # To detect a different color, change these ranges
        self.lower_pink = np.array([140, 150, 50])
        self.upper_pink = np.array([180, 255, 255])
        
        # Start the camera
        self.picam2.start()

    def take_pic(self):
        """
        Captures a single frame from the camera and returns it as an image array.
        """
        image = self.picam2.capture_array()
        return image

    
    def show_video(self, image):
        """
        Displays the image in a window named 'Live'.
        
        cv2.waitKey(1) is required to refresh the window.
        """
        cv2.imshow("Live", image)
        cv2.waitKey(1)
    
    def find_ball(self, image):
        """
        Detects a pink-colored ball in the given image.
        
        Returns:
            (x, y, area)
            x, y = position of the ball relative to image center
            area = size of detected object
            
            If no ball is found:
            returns (-1, -1, 0)
        """

        # Convert image from BGR (OpenCV default) to HSV color space
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create a mask that keeps only pixels within the pink color range
        mask = cv2.inRange(image_hsv, self.lower_pink, self.upper_pink)

        # Find contours (outlines of detected regions)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # If any contours are found
        if contours:
            # Select the largest contour (assumes the ball is the biggest pink object)
            largest_contour = max(contours, key=cv2.contourArea)

            # Find the smallest circle that can enclose the contour
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)

            # Calculate area of the contour
            area = cv2.contourArea(largest_contour)

            # Ignore very small detections (noise)
            # Increase this threshold if you get false detections
            if area > 200:

                # Draw a green circle around the detected object
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)

                # Show the processed image
                self.show_video(image)

                # Diameter of the detected object
                d = radius * 2

                # Approximate distance calculation (VERY rough!)
                # You may need to calibrate this for real-world use
                h = 10000 / d

                # Convert coordinates so that (0,0) is at image center
                # Instead of top-left corner
                x -= self.height / 2
                y -= self.width / 2

                # Swap axes and invert one axis
                # This is often done to match robot coordinate systems
                x, y = -y, x

                # Return position and size
                return int(x), int(y), int(area)

        # If no valid object is found, still show the video
        self.show_video(image)

        # Return default values indicating "no detection"
        return -1, -1, 0


    def clean_up_cam(self):
        """
        Properly shuts down the camera and closes all OpenCV windows.
        
        IMPORTANT:
        Always call this when exiting your program to avoid camera issues.
        """
        self.picam2.stop()           # Stop camera stream
        self.picam2.close()          # Release camera resources
        cv2.destroyAllWindows()      # Close all OpenCV display windows