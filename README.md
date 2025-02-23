# SketchUR3

**UR3tist** transforms the UR3 robotic arm into an artist, using ROS and OpenCV to recreate images on paper.

## How It Works

1. **Keypoint Detection**
   - Preprocesses images using grayscale conversion and edge detection.
   - Filters contours to extract keypoints for drawing.

2. **Coordinate Transformation**
   - Maps image keypoints to world coordinates using scaling factors.

3. **Drawing**
   - Commands the UR3 robotic arm to trace the keypoints with precise movements.
   - Ensures safe lifting between contours to avoid overlapping lines.

![SketchUR3 in Action](demo.gif)
