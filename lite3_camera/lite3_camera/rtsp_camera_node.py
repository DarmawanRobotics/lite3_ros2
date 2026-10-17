#!/usr/bin/env python3
"""
RTSP camera node for the Lite3's wide-angle front camera.

Reads rtsp://<motion host>:8554/test through GStreamer (Jetson hardware decoding when present),
reconnects on its own, rectifies the image when a calibration file is given, and publishes
image_raw (bgr8), image_raw/compressed (JPEG) and camera_info in camera_color_optical_frame.
"""

import threading
import time

import cv2
from lite3_camera.calibration import fov_intrinsics, info_values, rectifier, try_load
from lite3_camera.pipeline import decoder_candidates, gstreamer_pipeline, opencv_has_gstreamer
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class RtspCameraNode(Node):
    """One RTSP stream as a ROS camera."""

    def __init__(self, **kwargs):
        """Declare parameters and start the capture thread."""
        super().__init__('rtsp_camera', **kwargs)
        declare = self.declare_parameter
        declare('url', 'rtsp://192.168.1.120:8554/test')
        declare('codec', 'h264')
        declare('decoder', 'auto')
        declare('latency_ms', 0)
        declare('protocols', 'tcp')
        declare('width', 0)
        declare('height', 0)
        declare('max_fps', 15.0)
        declare('frame_id', 'camera_color_optical_frame')
        declare('calibration_file', '')
        declare('alpha', 0.0)
        declare('default_hfov_deg', 120.0)
        declare('publish_raw', True)
        declare('publish_compressed', True)
        declare('jpeg_quality', 75)
        declare('compressed_fps', 10.0)
        declare('latency_sec', 0.1)
        declare('reconnect_sec', 2.0)
        value = lambda name: self.get_parameter(name).value  # noqa: E731

        self._url = value('url')
        self._codec = value('codec')
        self._decoders = decoder_candidates(value('decoder'))
        self._latency_ms = int(value('latency_ms'))
        self._protocols = value('protocols')
        self._size = (int(value('width')), int(value('height')))
        self._min_period = 1.0 / value('max_fps') if value('max_fps') > 0.0 else 0.0
        self._frame_id = value('frame_id')
        self._alpha = float(value('alpha'))
        self._hfov = float(value('default_hfov_deg'))
        self._publish_raw = bool(value('publish_raw'))
        self._publish_compressed = bool(value('publish_compressed'))
        self._jpeg_quality = int(value('jpeg_quality'))
        self._compressed_period = (
            1.0 / value('compressed_fps') if value('compressed_fps') > 0.0 else 0.0
        )
        self._latency = Duration(seconds=float(value('latency_sec')))
        self._reconnect = float(value('reconnect_sec'))

        self._calibration, why = try_load(value('calibration_file'))
        if self._calibration is None:
            self.get_logger().warn(
                f'no camera calibration ({why}): camera_info assumes a {self._hfov:.0f} deg '
                'pinhole without distortion; AprilTag poses need a real calibration'
            )
        self._rectify = None   # (size, new K, maps), built for the first frame size
        self._info = None
        self._last_frame = 0.0
        self._last_jpeg = 0.0
        self.frames = 0
        self.decoder = ''

        self._raw_pub = self.create_publisher(Image, 'image_raw', 2)
        self._jpeg_pub = self.create_publisher(CompressedImage, 'image_raw/compressed', 2)
        self._info_pub = self.create_publisher(CameraInfo, 'camera_info', 2)

        self._running = threading.Event()
        self._thread = None
        if not self._url:
            self.get_logger().error('no url: nothing to read')
            return
        if not opencv_has_gstreamer(cv2.getBuildInformation()):
            self.get_logger().warn(
                'this OpenCV has no GStreamer (a pip opencv?): only the FFmpeg reader is left'
            )
        self._running.set()
        self._thread = threading.Thread(target=self.capture_loop, name='rtsp', daemon=True)
        self._thread.start()

    def open_stream(self):
        """Open the stream with the first decoder that works; None when none does."""
        for decoder in self._decoders:
            if decoder == 'ffmpeg':
                capture = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
            else:
                pipeline = gstreamer_pipeline(
                    self._url, self._codec, decoder, self._latency_ms, self._protocols
                )
                capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if capture.isOpened():
                if decoder != self.decoder:
                    self.get_logger().info(f'{self._url} open ({decoder} decoder)')
                self.decoder = decoder
                self._decoders.remove(decoder)
                self._decoders.insert(0, decoder)  # try it first on the next reconnect
                return capture
            capture.release()
        return None

    def capture_loop(self):
        """Read frames until stopped; reconnect when the stream drops."""
        warned = False
        while self._running.is_set():
            capture = self.open_stream()
            if capture is None:
                if not warned:
                    self.get_logger().warn(
                        f'cannot open {self._url}, retrying every {self._reconnect:.0f} s'
                    )
                    warned = True
                time.sleep(self._reconnect)
                continue
            warned = False
            while self._running.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    self.get_logger().warn('stream lost, reconnecting')
                    break
                self.process_frame(frame)
            capture.release()
            time.sleep(min(self._reconnect, 1.0))

    def process_frame(self, frame: np.ndarray):
        """Resize, rectify and publish one BGR frame."""
        now = time.monotonic()
        if self._min_period and now - self._last_frame < self._min_period:
            return
        self._last_frame = now
        width, height = self._size
        if width > 0 and height > 0 and frame.shape[1::-1] != (width, height):
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        frame = self.rectified(frame)
        stamp = (self.get_clock().now() - self._latency).to_msg()
        self.frames += 1
        if self._publish_raw:
            self._raw_pub.publish(self.image_msg(frame, stamp))
        if self._publish_compressed and now - self._last_jpeg >= self._compressed_period:
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
            if ok:
                self._last_jpeg = now
                msg = CompressedImage(format='jpeg', data=jpeg.tobytes())
                msg.header.stamp, msg.header.frame_id = stamp, self._frame_id
                self._jpeg_pub.publish(msg)
        info = self.camera_info(frame.shape[1], frame.shape[0])
        info.header.stamp = stamp
        self._info_pub.publish(info)

    def rectified(self, frame: np.ndarray) -> np.ndarray:
        """Undistort the frame with the calibration, scaled to the frame size."""
        if self._calibration is None:
            return frame
        size = (frame.shape[1], frame.shape[0])
        if self._rectify is None or self._rectify[0] != size:
            new_k, maps = rectifier(self._calibration.scaled(*size), self._alpha)
            self._rectify = (size, new_k, maps)
            self._info = None
        _, _, (map1, map2) = self._rectify
        return cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

    def camera_info(self, width: int, height: int) -> CameraInfo:
        """Return camera_info of the published (rectified) image."""
        if self._info is None or (self._info.width, self._info.height) != (width, height):
            if self._rectify is not None and self._rectify[0] == (width, height):
                k = self._rectify[1]
            else:
                k = fov_intrinsics(width, height, self._hfov)
            values = info_values(k, width, height)
            info = CameraInfo()
            info.width, info.height = values['width'], values['height']
            info.distortion_model = values['distortion_model']
            info.d, info.k, info.r, info.p = values['d'], values['k'], values['r'], values['p']
            info.header.frame_id = self._frame_id
            self._info = info
        return self._info

    def image_msg(self, frame: np.ndarray, stamp) -> Image:
        """Wrap a BGR frame as sensor_msgs/Image without cv_bridge."""
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height, msg.width = int(frame.shape[0]), int(frame.shape[1])
        msg.encoding = 'bgr8'
        msg.step = int(frame.shape[1]) * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        return msg

    def destroy_node(self):
        """Stop the capture thread before the node goes."""
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    """Spin the camera node."""
    rclpy.init(args=args)
    node = RtspCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise  # a real error while running
        # else: Ctrl-C shut the context down under spin(), a normal stop
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
