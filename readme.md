## Installation
```
pip install pyrealsense2
```

## Usage
```
python realsense_camera_node.py
```

## Depth Image ROS Topic
- `/camera/depth/image_rect_raw`

## Filter Configuration
To adjust the depth image filters, modify the filter parameters in the `start_pipeline` method of the `VisualHandlerNode` class.

For detailed filter documentation, refer to: https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md