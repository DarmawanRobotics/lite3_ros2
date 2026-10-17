"""GStreamer pipelines for the RTSP camera (no ROS imports)."""

from typing import List

DECODERS = ('jetson', 'software', 'ffmpeg')
DEPAY = {'h264': 'rtph264depay ! h264parse', 'h265': 'rtph265depay ! h265parse'}
SOFTWARE = {'h264': 'avdec_h264', 'h265': 'avdec_h265'}


def gstreamer_pipeline(url: str, codec: str = 'h264', decoder: str = 'jetson',
                       latency_ms: int = 0, protocols: str = 'tcp') -> str:
    """Return the OpenCV GStreamer pipeline for an RTSP stream; BGR frames, newest only."""
    codec = codec.lower()
    if codec not in DEPAY:
        raise ValueError(f'codec must be h264 or h265, not {codec!r}')
    if decoder == 'jetson':
        decode = 'nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert'
    elif decoder == 'software':
        decode = f'{SOFTWARE[codec]} ! videoconvert'
    else:
        raise ValueError(f'no GStreamer pipeline for decoder {decoder!r}')
    return (
        f'rtspsrc location={url} latency={int(latency_ms)} protocols={protocols} ! '
        f'{DEPAY[codec]} ! {decode} ! video/x-raw,format=BGR ! '
        'appsink drop=true max-buffers=1 sync=false'
    )


def decoder_candidates(decoder: str) -> List[str]:
    """Return the decoders to try in order: auto tries the Jetson, then software, then FFmpeg."""
    decoder = decoder.lower()
    if decoder == 'auto':
        return list(DECODERS)
    if decoder not in DECODERS:
        raise ValueError(f'decoder must be auto, {", ".join(DECODERS)}, not {decoder!r}')
    return [decoder]


def opencv_has_gstreamer(build_information: str) -> bool:
    """Return True when OpenCV's build information lists GStreamer support."""
    for line in build_information.splitlines():
        if line.strip().startswith('GStreamer:'):
            return 'YES' in line.upper()
    return False
