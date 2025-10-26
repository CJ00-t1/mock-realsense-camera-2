# Mock RealSense Camera

## Usage

```bash
ros2 launch mock_realsense_camera launch.py
```

## Topics

|Topic|Msg|
|---|---|
|'/camera/color/image_raw'|Image|
|'/camera/aligned_depth_to_color/image_raw'|Image|
|'/camera/color/camera_info'|CameraInfo|
|'/camera/aligned_depth_to_color/camera_info'|CameraInfo|

These publish images based on the files in `data/rgb` and `data/depth`. All input images must be PNGs. Additionally, camera info is read from `config/realsense_camer_info.yaml`. 
