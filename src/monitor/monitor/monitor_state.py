from datetime import datetime, timezone
import threading
import time


class MonitorState:
    def __init__( self, stale_timeout_sec, image_source_width, image_source_height,
                  debug_keys=None ):
        self._lock = threading.Lock()
        self._stale_timeout_sec = stale_timeout_sec

        # 토픽 수신율(Hz) 미터: 이름별 최근 도착 시각(monotonic) 목록.
        # 최근 _rate_window_sec 창의 도착 간격 평균으로 Hz를 낸다(ros2 topic hz 방식).
        self._rate_window_sec = 5.0
        self._rate_events = {}

        self._battery_status = None
        self._battery_updated_at = None
        self._battery_updated_monotonic = None

        self._image_frame = None
        self._image_width = image_source_width
        self._image_height = image_source_height
        self._image_updated_at = None
        self._image_updated_monotonic = None
        # 디버그 채널은 config(DEBUG_CHANNELS)로 정해지므로 키 목록을 받아 동적 초기화한다.
        self._debug_keys = list(debug_keys or [])
        self._debug_frames = {key: None for key in self._debug_keys}
        self._debug_widths = {key: image_source_width for key in self._debug_keys}
        self._debug_heights = {key: image_source_height for key in self._debug_keys}
        self._debug_updated_at = {key: None for key in self._debug_keys}
        self._debug_updated_monotonic = {key: None for key in self._debug_keys}

        self._throttle = None
        self._steering = None
        self._control_updated_at = None
        self._control_updated_monotonic = None

        # 인지→제어 핸드오프 경로(/perception/lane_path). detect()가 edge+sliding
        # window+centerline을 한 콜백에서 계산·발행하므로 이 토픽 rate = 차선 인지 처리율.
        self._lane_path_num_points = None
        self._lane_path_updated_at = None
        self._lane_path_updated_monotonic = None

        self._is_recording = False
        self._recording_updated_at = None
        self._recording_updated_monotonic = None

        self._storage_used_percentage = None
        self._storage_used_bytes = None
        self._storage_total_bytes = None
        self._storage_updated_at = None
        self._storage_updated_monotonic = None

    def _is_stale(self, updated_monotonic):
        if updated_monotonic is None:
            return True

        return (time.monotonic() - updated_monotonic) > self._stale_timeout_sec

    def _record_rate_locked(self, name, now_monotonic):
        """@brief 수신 이벤트 1건 기록(호출자가 이미 lock 보유). 창 밖 샘플은 정리."""
        events = self._rate_events.get(name)
        if events is None:
            events = []
            self._rate_events[name] = events
        events.append(now_monotonic)
        cutoff = now_monotonic - self._rate_window_sec
        while events and events[0] < cutoff:
            events.pop(0)

    def _hz_locked(self, name):
        """@brief 최근 창의 도착 간격 평균으로 Hz 산출(호출자가 lock 보유). 샘플 부족 시 None."""
        events = self._rate_events.get(name)
        if not events:
            return None
        cutoff = time.monotonic() - self._rate_window_sec
        recent = [t for t in events if t >= cutoff]
        if len(recent) < 2:
            return None
        span = recent[-1] - recent[0]
        if span <= 0.0:
            return None
        return (len(recent) - 1) / span

    def _format_gb(self, size_bytes):
        if size_bytes is None:
            return '--'

        return f'{size_bytes / (1024 ** 3):.1f} GB'

    def update_battery(self, battery_status):
        clamped_value = max(0.0, min(100.0, float(battery_status)))

        with self._lock:
            self._battery_status = clamped_value
            self._battery_updated_at = datetime.now(timezone.utc)
            self._battery_updated_monotonic = time.monotonic()

    def update_image(self, frame_bytes, source_width, source_height):
        with self._lock:
            self._image_frame = frame_bytes
            self._image_width = int(source_width)
            self._image_height = int(source_height)
            self._image_updated_at = datetime.now(timezone.utc)
            self._image_updated_monotonic = time.monotonic()

    def update_control(self, throttle, steering):
        clamped_throttle = max(-1.0, min(1.0, float(throttle)))
        clamped_steering = max(-1.0, min(1.0, float(steering)))

        with self._lock:
            self._throttle = clamped_throttle
            self._steering = clamped_steering
            self._control_updated_at = datetime.now(timezone.utc)
            self._control_updated_monotonic = time.monotonic()

    def update_debug_image(self, image_key, frame_bytes, source_width, source_height):
        if image_key not in self._debug_frames:
            return

        with self._lock:
            self._debug_frames[image_key] = frame_bytes
            self._debug_widths[image_key] = int(source_width)
            self._debug_heights[image_key] = int(source_height)
            self._debug_updated_at[image_key] = datetime.now(timezone.utc)
            self._debug_updated_monotonic[image_key] = time.monotonic()

    def update_recording(self, is_recording):
        with self._lock:
            self._is_recording = bool(is_recording)
            self._recording_updated_at = datetime.now(timezone.utc)
            self._recording_updated_monotonic = time.monotonic()

    def update_storage(self, used_bytes, total_bytes):
        if total_bytes <= 0:
            return

        used_percentage = (float(used_bytes) / float(total_bytes)) * 100.0

        with self._lock:
            self._storage_used_percentage = max(0.0, min(100.0, used_percentage))
            self._storage_used_bytes = int(used_bytes)
            self._storage_total_bytes = int(total_bytes)
            self._storage_updated_at = datetime.now(timezone.utc)
            self._storage_updated_monotonic = time.monotonic()

    def get_latest_frame(self):
        with self._lock:
            return self._image_frame

    def get_debug_frame(self, image_key):
        with self._lock:
            return self._debug_frames.get(image_key)

    def snapshot(self):
        with self._lock:
            battery_status = self._battery_status
            battery_updated_at = self._battery_updated_at
            battery_updated_monotonic = self._battery_updated_monotonic

            image_width = self._image_width
            image_height = self._image_height
            image_updated_at = self._image_updated_at
            image_updated_monotonic = self._image_updated_monotonic
            debug_widths = dict(self._debug_widths)
            debug_heights = dict(self._debug_heights)
            debug_updated_at = dict(self._debug_updated_at)
            debug_updated_monotonic = dict(self._debug_updated_monotonic)

            throttle = self._throttle
            steering = self._steering
            control_updated_at = self._control_updated_at
            control_updated_monotonic = self._control_updated_monotonic

            is_recording = self._is_recording
            recording_updated_at = self._recording_updated_at
            recording_updated_monotonic = self._recording_updated_monotonic

            storage_used_percentage = self._storage_used_percentage
            storage_used_bytes = self._storage_used_bytes
            storage_total_bytes = self._storage_total_bytes
            storage_updated_at = self._storage_updated_at
            storage_updated_monotonic = self._storage_updated_monotonic

        battery_has_data = battery_status is not None
        image_has_data = image_updated_at is not None
        control_has_data = throttle is not None and steering is not None
        storage_has_data = (
            storage_used_percentage is not None
            and storage_used_bytes is not None
            and storage_total_bytes is not None
        )
        debug_image = {}
        for key in self._debug_keys:
            updated_at = debug_updated_at[key]
            debug_image[key] = {
                'has_data': updated_at is not None,
                'updated_at': None if updated_at is None else updated_at.isoformat(),
                'is_stale': self._is_stale(debug_updated_monotonic[key]),
                'resolution_display': f"{debug_widths[key]}x{debug_heights[key]}",
            }

        return {
            'battery': {
                'has_data': battery_has_data,
                'battery_status': None if battery_status is None else round(battery_status, 1),
                'battery_display': '--.-%' if battery_status is None else f'{battery_status:.1f}%',
                'updated_at': None if battery_updated_at is None else battery_updated_at.isoformat(),
                'is_stale': self._is_stale(battery_updated_monotonic),
            },
            'image': {
                'has_data': image_has_data,
                'updated_at': None if image_updated_at is None else image_updated_at.isoformat(),
                'is_stale': self._is_stale(image_updated_monotonic),
                'resolution_display': f'{image_width}x{image_height}',
            },
            'control': {
                'has_data': control_has_data,
                'updated_at': None if control_updated_at is None else control_updated_at.isoformat(),
                'is_stale': self._is_stale(control_updated_monotonic),
                'throttle': None if throttle is None else round(throttle, 2),
                'steering': None if steering is None else round(steering, 2),
            },
            'recording': {
                'has_data': recording_updated_at is not None,
                'updated_at': None if recording_updated_at is None else recording_updated_at.isoformat(),
                'is_stale': self._is_stale(recording_updated_monotonic),
                'is_recording': bool(is_recording),
            },
            'storage': {
                'has_data': storage_has_data,
                'updated_at': None if storage_updated_at is None else storage_updated_at.isoformat(),
                'is_stale': self._is_stale(storage_updated_monotonic),
                'used_percentage': (
                    None if storage_used_percentage is None else round(storage_used_percentage, 1)
                ),
                'used_display': (
                    '--.-%' if storage_used_percentage is None
                    else f'{storage_used_percentage:.1f}%'
                ),
                'used_space_display': self._format_gb(storage_used_bytes),
                'total_space_display': self._format_gb(storage_total_bytes),
            },
            'debug_image': debug_image,
        }
