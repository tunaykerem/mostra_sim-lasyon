# Cone Detection Configuration

# Detection methods to use:
# "color": Use color-based detection for yellow cones
# "yolo": Use YOLO model detection (requires trained model with "cone" class)
# "both": Use both methods (recommended for best accuracy)
# Note: YOLOv8 detector will automatically handle cone detection if "cone" or "koni" class is present
DETECTION_METHODS = ["both"]  # Using both since last.pt has "koni" class + color backup

# Color-based detection parameters (for yellow cones)
COLOR_DETECTION = {
    "lower_hsv": [20, 100, 100],    # Lower HSV threshold for yellow
    "upper_hsv": [30, 255, 255],    # Upper HSV threshold for yellow
    "min_contour_area": 500,        # Minimum contour area to filter noise
    "focal_length": 800,            # Camera focal length in pixels
    "cone_height": 0.3              # Real cone height in meters
}

# Obstacle parameters
OBSTACLE_CONFIG = {
    "cone_width": 0.3,              # Cone width for obstacle boundaries (meters)
    "max_detection_distance": 20.0, # Maximum detection distance (meters)
    "min_detection_distance": 0.5   # Minimum detection distance (meters)
}

# Debug settings
DEBUG = {
    "show_detection_info": True,    # Print detection information
    "show_obstacle_boundaries": True, # Draw obstacle boundaries on frame
    "show_trajectory_updates": True   # Print when trajectory is updated
}
