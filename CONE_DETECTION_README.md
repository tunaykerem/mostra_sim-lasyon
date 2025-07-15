# Yellow Cone Detection System

This system adds real-time yellow cone detection to the autonomous driving simulation, enabling the vehicle to detect and avoid physical obstacles like traffic cones.

## Current Configuration

**🎯 YOLOv8 with last.pt weights + Color-based cone detection**

The system now uses YOLOv8 with `last.pt` weights for traffic sign detection, combined with color-based detection for yellow cones.

## How It Works

The system has been enhanced with multiple detection methods:

### 1. YOLOv8 Detection (Primary for Traffic Signs)
- **File**: `detection/yolov8_detector.py`
- **Model**: Uses `last.pt` weights
- **Method**: Deep learning neural network detection
- **Advantages**: High accuracy, robust to lighting changes
- **Supports**: Traffic signs and any trained object classes (including cones if trained)

### 2. Color-Based Detection (Primary for Cones)
- **File**: `detection/cone_detector.py`
- **Method**: HSV color space filtering to detect yellow objects
- **Advantages**: Works immediately without training, good for yellow cones
- **Configuration**: Adjustable HSV thresholds in `detection/cone_config.py`

### 3. Legacy YOLO v4 Detection (Backup)
- **File**: `detection/detect.py`
- **Method**: Uses YOLO v4 with custom weights
- **Status**: Available as fallback option

## System Integration

### Detection Pipeline
1. **Frame Capture**: Camera frames are processed in `thread_function_1`
2. **Sign Detection**: YOLO processes frame for traffic signs
3. **Cone Detection**: Color-based detector processes frame for cones
4. **Obstacle Update**: Detected cones update `Global.dubaEngel`
5. **Trajectory Update**: If obstacles change, trajectory is regenerated
6. **Path Planning**: `buyukDuba.dubaEngel()` handles obstacle avoidance

### Key Components

#### Modified Files:
- `main3.py`: Now uses YOLOv8 detector with `last.pt` weights
- `detection/yolov8_detector.py`: New YOLOv8 wrapper for `last.pt`
- `webcam1.py`: Fixed imports for YOLOv8 support
- `detection/cone_config.py`: Updated to use both detection methods

#### New Files:
- `detection/yolov8_detector.py`: YOLOv8 detector implementation
- `detection/cone_detector.py`: Color-based cone detection implementation
- `detection/cone_config.py`: Configuration settings
- `inspect_model.py`: Tool to check what classes are in your model

#### Enhanced Files:
- `detection/detect.py`: Enhanced YOLO v4 detector with cone support (backup)
- `detection/detector/obj.names`: Added "cone" class for future training

#### Existing Obstacle System (Enhanced):
- `FinalPathGenerator/NesneTespit/buyukDuba.py`: Obstacle avoidance logic
- `FinalPathGenerator/util/Global.py`: Global state management
- `FinalPathGenerator/Main.py`: Trajectory generation with obstacles

## Configuration

Edit `detection/cone_config.py` to adjust:

### Detection Methods
```python
DETECTION_METHODS = ["both"]  # Options: ["color"], ["yolo"], ["both"]
```
**Current setting: "both"** - Uses YOLOv8 for traffic signs and color detection for cones

### Color Detection Parameters
```python
COLOR_DETECTION = {
    "lower_hsv": [20, 100, 100],    # Lower HSV for yellow
    "upper_hsv": [30, 255, 255],    # Upper HSV for yellow
    "min_contour_area": 500,        # Minimum size to detect
    "focal_length": 800,            # Camera focal length
    "cone_height": 0.3              # Real cone height (30cm)
}
```

### Obstacle Settings
```python
OBSTACLE_CONFIG = {
    "cone_width": 0.3,              # Cone width for path planning
    "max_detection_distance": 20.0, # Max detection range
    "min_detection_distance": 0.5   # Min detection range
}
```

## Usage

### For Yellow Cones (Default)
1. Ensure cones are bright yellow and well-lit
2. Run the simulation normally
3. System will automatically detect and avoid cones

### For Better Accuracy (Optional)
1. Collect training images of cones in various conditions
2. Retrain YOLO model to include "cone" class
3. Update model weights
4. Change config to use YOLO: `DETECTION_METHODS = ["yolo"]`

## Testing

### Visual Feedback
- **Green boxes**: Detected cones with distance info
- **Red text**: "OBSTACLE DETECTED" when cone is found
- **Console output**: Detection coordinates and trajectory updates

### Expected Behavior
1. **Approach cone**: Vehicle detects cone ahead
2. **Trajectory update**: New path generated to avoid cone
3. **Avoidance maneuver**: Vehicle steers around obstacle
4. **Clear path**: When cone is passed, normal trajectory resumes

## Troubleshooting

### Cone Not Detected
- Check lighting conditions (yellow should be bright)
- Adjust HSV thresholds in `cone_config.py`
- Ensure cone is large enough (check `min_contour_area`)
- Verify camera feed shows yellow objects clearly

### False Detections
- Increase `min_contour_area` to filter small yellow objects
- Narrow HSV range to be more specific to cone color
- Add shape filtering (circular/triangular detection)

### Vehicle Doesn't Avoid Obstacle
- Check console for "Obstacle detected" and "Trajectory updated" messages
- Verify `Global.dubaEngel` contains valid obstacle coordinates
- Ensure `buyukDuba.dubaEngel()` is being called in trajectory generation

### Performance Issues
- Reduce detection frequency if needed
- Optimize HSV processing area (crop frame to road region)
- Use YOLO detection only if color detection is too slow

## Future Enhancements

1. **Multi-cone Detection**: Handle multiple obstacles simultaneously
2. **Cone Classification**: Distinguish between different obstacle types
3. **Distance Calibration**: Improve distance estimation accuracy
4. **Lighting Adaptation**: Auto-adjust HSV thresholds based on conditions
5. **Shape Validation**: Add cone shape verification to reduce false positives
6. **LiDAR Integration**: Combine vision with LiDAR for better 3D understanding

## Technical Details

### Coordinate Systems
- **Camera coordinates**: (x=horizontal, y=forward) from vehicle perspective
- **Obstacle format**: `Global.dubaEngel = (left_point, right_point)`
- **Point format**: `[forward_distance, side_offset, height]`

### Integration Points
- **Detection**: Thread 1 processes camera frames
- **Planning**: Obstacle data triggers trajectory regeneration
- **Control**: Thread 2 follows updated trajectory
- **Communication**: Event-based signaling between threads

This system provides a robust foundation for obstacle detection and avoidance, with room for future enhancements based on specific requirements and testing results.
