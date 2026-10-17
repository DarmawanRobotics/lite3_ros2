"""GStreamer pipeline strings and decoder order."""

from lite3_camera.pipeline import decoder_candidates, gstreamer_pipeline, opencv_has_gstreamer
import pytest

URL = 'rtsp://192.168.1.120:8554/test'


def test_jetson_pipeline_uses_the_hardware_decoder():
    """The Jetson pipeline decodes with nvv4l2decoder and hands BGR frames to the appsink."""
    pipeline = gstreamer_pipeline(URL, 'h264', 'jetson', latency_ms=50)
    assert pipeline.startswith(f'rtspsrc location={URL} latency=50 protocols=tcp')
    assert 'rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv' in pipeline
    assert pipeline.endswith('video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false')


def test_software_pipeline_and_h265():
    """The software pipeline uses libav; h265 changes the depayloader too."""
    pipeline = gstreamer_pipeline(URL, 'H265', 'software')
    assert 'rtph265depay ! h265parse ! avdec_h265 ! videoconvert' in pipeline
    with pytest.raises(ValueError):
        gstreamer_pipeline(URL, 'mjpeg', 'software')
    with pytest.raises(ValueError):
        gstreamer_pipeline(URL, 'h264', 'ffmpeg')


def test_decoder_order():
    """The auto mode tries the Jetson first and FFmpeg last; an explicit choice stays alone."""
    assert decoder_candidates('auto') == ['jetson', 'software', 'ffmpeg']
    assert decoder_candidates('software') == ['software']
    with pytest.raises(ValueError):
        decoder_candidates('cuda')


def test_opencv_gstreamer_detection():
    """The build information tells whether OpenCV can run a GStreamer pipeline."""
    assert opencv_has_gstreamer('  Video I/O:\n    GStreamer:                   YES (1.20.3)\n')
    assert not opencv_has_gstreamer('    GStreamer:                   NO\n')
    assert not opencv_has_gstreamer('')
