# lite3_camera

`rtsp_camera_node`: the Lite3's own wide-angle front camera as a ROS camera. The motion host
streams it at `rtsp://<motion host>:8554/test` (Deep Robotics *UDP Commands* 1.2.11:
`192.168.1.120` or `192.168.2.1`).

```bash
ros2 run lite3_camera rtsp_camera_node --ros-args \
  --params-file $(ros2 pkg prefix lite3_camera)/share/lite3_camera/config/camera_params.yaml \
  -r image_raw:=/camera/camera/color/image_raw \
  -r image_raw/compressed:=/camera/camera/color/image_raw/compressed \
  -r camera_info:=/camera/camera/color/camera_info
```

`lite3_bringup` starts it with these remappings (`camera:=rtsp`), so Molly sees the camera where
the robot package contract wants it.

- **Decoding**: GStreamer through OpenCV. `decoder: auto` tries the Jetson hardware decoder
  (`nvv4l2decoder`), then software (`avdec_h264`, gstreamer1.0-libav), then OpenCV's FFmpeg
  reader; the one that worked is tried first on every reconnect. The appsink keeps only the
  newest frame, so a slow consumer never builds up delay.
- **Reconnects** every `reconnect_sec` when the stream cannot be opened or drops.
- **Rectification**: with `calibration_file` (camera_calibration YAML, `plumb_bob`,
  `rational_polynomial` or fisheye `equidistant`) the published image is undistorted and
  `camera_info` describes the ideal pinhole camera (no distortion), which is what the AprilTag
  detector expects. Without it `camera_info` assumes `default_hfov_deg`; tag poses are then only
  approximate. Calibrate once: [config/calibration.example.yaml](config/calibration.example.yaml).

| Topic | Type | |
|---|---|---|
| `image_raw` | `sensor_msgs/Image` | bgr8, rectified when calibrated, `max_fps` |
| `image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG, `compressed_fps`, `jpeg_quality` |
| `camera_info` | `sensor_msgs/CameraInfo` | of the published image |

All in `frame_id` `camera_color_optical_frame`, stamped with the receive time minus
`latency_sec`.

Parameters: [config/camera_params.yaml](config/camera_params.yaml).

**OpenCV needs GStreamer.** The Ubuntu / JetPack `python3-opencv` has it; a pip `opencv-python`
does not and hides the system one (`pip uninstall opencv-python`). The node logs a warning when
GStreamer is missing; `lite3_bringup/scripts/lite3_setup.sh check` checks it too.
