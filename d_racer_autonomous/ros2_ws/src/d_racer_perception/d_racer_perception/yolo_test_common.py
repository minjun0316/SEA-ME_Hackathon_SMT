from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


def decode_compressed_image(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def encode_jpeg(frame, header, quality=80):
    params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    ok, enc = cv2.imencode('.jpg', frame, params)
    if not ok:
        return None

    out = CompressedImage()
    out.header = header
    out.format = 'jpeg'
    out.data = enc.tobytes()
    return out


def resolve_model_path(model_path, package_name='d_racer_perception'):
    path = Path(str(model_path)).expanduser()
    if path.is_absolute() and path.exists():
        return path
    if path.exists():
        return path.resolve()

    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(Path(get_package_share_directory(package_name)) / 'models' / path.name)
    except Exception:
        pass

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidates.append(parent / 'models' / path.name)
        candidates.append(parent.parent / 'models' / path.name)

    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()

    return path


class RateMeter:
    def __init__(self):
        self.last_time = None

    def tick(self):
        now = perf_counter()
        if self.last_time is None:
            self.last_time = now
            return 0.0

        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0:
            return 0.0
        return 1.0 / dt
