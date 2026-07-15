"""@file mission_cues_node.py
@brief 미션 신호 인지 노드 → `/perception/mission_cues`(racer_msgs/MissionCues) 발행.

@details
차선 기하 외의 미션 페이즈 전환 신호(신호등·체커보드·빨강구역·아루코)를 한 토픽으로
묶어 발행한다(계약: docs/interfaces.md §4.5). 신호는 판단(mission)이 소비.

현재 구현: **ArUco 마커만**(cv2.aruco, 학습 불필요). 나머지(traffic_light/checkerboard/
red_zone)는 **stub**(기본값 발행) — 초록불(색검출)·빨강구역은 후속 추가.

@par 구독 → 발행
- 구독 `camera/image/compressed`(sensor_msgs/CompressedImage): 키트 camera_node.
- 발행 `/perception/mission_cues`(racer_msgs/MissionCues): reliable depth1(계약).
- (옵션) 발행 `perception/mission_cues/aruco/debug/compressed`: 오버레이.

@note 대회 시나리오: 심판이 막대에 붙인 ArUco(ID=3)를 **멀리서** 보여주면 정지,
      마커가 사라지면 재출발. → 하단 ROI 아님, **화면 전체**에서 ID 3만 인정.
      마커가 잠깐 흔들려 놓쳐도 튀지 않게 hold(디바운스)로 present 유지.
"""
from __future__ import annotations

import os

# 4코어 보드 CPU 경합 완화: YOLO(NCNN/torch) 네이티브 스레드풀 상한을 ultralytics
# import 前에 고정한다(TrafficLightDetector가 지연 import). aruco 경로엔 영향 없음.
os.environ.setdefault('OMP_NUM_THREADS', '2')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import sys
from pathlib import Path as _FsPath


def _find_core_root() -> str:
    """@brief core import 루트 탐색. lane_detect_node/controller_node와 동일 규약."""
    env_root = os.environ.get('D_RACER_ROOT')
    candidates = []
    if env_root:
        candidates.append(_FsPath(os.path.expanduser(env_root)))
    here = _FsPath(__file__).resolve()
    for base in [here, *here.parents]:
        candidates.append(base)
        candidates.append(base / 'd_racer_autonomous')
    for cand in candidates:
        if (cand / 'core' / '__init__.py').exists() and \
           (cand / 'config' / 'vehicle.yaml').exists():
            return str(cand)
    raise RuntimeError(
        'mission_cues_node: core 패키지를 찾지 못했습니다. 환경변수 D_RACER_ROOT 로 '
        'd_racer_autonomous 경로를 지정하세요.')


_CORE_ROOT = _find_core_root()
if _CORE_ROOT not in sys.path:
    sys.path.insert(0, _CORE_ROOT)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402
from rclpy.time import Time  # noqa: E402

from sensor_msgs.msg import CompressedImage  # noqa: E402
from racer_msgs.msg import MissionCues, LaneMode  # noqa: E402

from core.perception.aruco_detect import ArucoDetector, ArucoConfig  # noqa: E402
from core.perception.traffic_light_detect import (  # noqa: E402
    TrafficLightDetector, TrafficCueConfig, TL_NONE, TL_RED, TL_GREEN)
from core.perception.sign_detect import (  # noqa: E402
    SignDetector, SignConfig, SIGN_NONE)

from .yolo_test_common import resolve_model_path  # noqa: E402


class MissionCuesNode(Node):
    """@brief compressed 영상 → ArUco 검출(+stub) → MissionCues 발행."""

    def __init__(self):
        super().__init__('mission_cues_node')

        # --- 파라미터(전부 YAML/CLI 조정) ---
        self.declare_parameter('image_topic', 'camera/image/compressed')
        self.declare_parameter('mission_cues_topic', '/perception/mission_cues')
        self.declare_parameter('lane_mode_topic', '/decision/lane_mode')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_jpeg_quality', 80)

        # ArUco 검출 파라미터.
        d = ArucoConfig()
        self.declare_parameter('aruco_dictionary', d.dictionary)  # 대회 마커에 맞춤(미확정).
        self.declare_parameter('aruco_roi_bottom_frac', 1.0)      # 막대에 들고 멀리서→전체.
        self.declare_parameter('aruco_min_perimeter_px', d.min_perimeter_px)
        # 정지 트리거로 인정할 마커 ID(대회=[3]). 비었거나 음수 포함이면 '아무 마커나'.
        self.declare_parameter('aruco_target_ids', [3])
        # present 홀드[s]: 마커 잠깐 놓쳐도 이 시간 안엔 유지(흔들림→튐 방지, 재출발 지연).
        self.declare_parameter('aruco_hold_sec', 0.4)

        target_ids = list(self.get_parameter('aruco_target_ids').value)
        # 음수/빈 리스트 = 아무 마커나 인정.
        use_any = (len(target_ids) == 0) or any(int(t) < 0 for t in target_ids)
        cfg = ArucoConfig(
            dictionary=str(self.get_parameter('aruco_dictionary').value),
            roi_bottom_frac=float(self.get_parameter('aruco_roi_bottom_frac').value),
            min_perimeter_px=float(self.get_parameter('aruco_min_perimeter_px').value),
            target_ids=() if use_any else tuple(int(t) for t in target_ids),
        )
        try:
            self.detector = ArucoDetector(cfg)
        except Exception as e:                        # 사전 이름 오타/미지원 등.
            self.get_logger().fatal(f'ArucoDetector 초기화 실패: {e}')
            raise

        self.hold_sec = float(self.get_parameter('aruco_hold_sec').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.debug_quality = int(self.get_parameter('debug_jpeg_quality').value)

        image_topic = str(self.get_parameter('image_topic').value)
        cues_topic = str(self.get_parameter('mission_cues_topic').value)

        reliable_q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)
        best_effort_q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                   history=HistoryPolicy.KEEP_LAST)

        self.pub_cues = self.create_publisher(MissionCues, cues_topic, reliable_q)
        self.pub_debug = None
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                CompressedImage, 'perception/mission_cues/aruco/debug/compressed', 1)

        self.sub = self.create_subscription(
            CompressedImage, image_topic, self.on_image, best_effort_q)

        # 디바운스 상태.
        self._last_seen_time: Time | None = None
        self._held_present = False
        self._frames = 0

        # --- YOLO 미션신호(신호등/체커보드) 검출 ---
        # mission_cues_node가 MissionCues 주인이라 신호등·체커보드(둘 다 YOLO)도 여기서
        # 채운다. ultralytics는 무거워 실패해도 aruco는 계속 동작하도록 격리.
        self.declare_parameter('yolo_enable', True)
        self.declare_parameter('yolo_model_path', 'best_ncnn_model')
        self.declare_parameter('yolo_conf', 0.35)
        self.declare_parameter('yolo_imgsz', 320)
        self.declare_parameter('yolo_max_infer_hz', 6.0)     # 추론 상한(카메라 다 안 돌림→CPU 경합↓).
        self.declare_parameter('yolo_cv_threads', 1)
        self.declare_parameter('yolo_torch_threads', 2)
        self.declare_parameter('yolo_stale_sec', 1.0)        # 이 시간 내 추론 없으면 신호 NONE.
        self.declare_parameter('tl_prefer_red', True)        # 초록·빨강 동시 → RED(출발 안전측).
        self.declare_parameter('tl_green_confirm_sec', 0.5)  # 초록 연속 확정 시간(오출발 방지).
        # [07-15] 빨강 연속 확정 시간(초록과 대칭). 빨강은 그동안 '확정 없이 즉시' 발행돼
        # 오검출 한 번이 코스를 끝냈다. 판단측 red_confirm_count(연속 N개)로는 못 막는다 —
        # 이 노드는 카메라 속도로 발행하면서 추론 결과를 yolo_stale_sec까지 재사용하므로,
        # 추론 1번이 같은 값 수십 개로 재발행돼 'N연속'을 스스로 충족시켜 버리기 때문.
        # 시간 기반 확정만이 재발행에 안 속는다. 종료가 늦으면 ↓, 오종료면 ↑.
        self.declare_parameter('tl_red_confirm_sec', 0.5)
        self.declare_parameter('checker_hold_sec', 0.3)      # (폐기) 체커보드 필드용 잔존값.
        # 통합 모델 클래스 id(신호등+팻말 한 모델). 기본=best.pt: 0 green/1 left/2 red/3 right.
        _tc = TrafficCueConfig()
        self.declare_parameter('tl_class_green', _tc.class_green)
        self.declare_parameter('tl_class_red', _tc.class_red)
        self.declare_parameter('tl_class_left', _tc.class_left)
        self.declare_parameter('tl_class_right', _tc.class_right)
        # 팻말 거리·각도 게이트(신호등엔 미적용). 멀리서/비스듬히 잡아 차선전환을 미리
        # 지시하는 것 방지. 로그의 h=/asp= 실측치를 보고 임계를 잡는다.
        self.declare_parameter('sign_min_box_h_frac', _tc.sign_min_box_h_frac)
        self.declare_parameter('sign_min_box_aspect', _tc.sign_min_box_aspect)
        # 빨강 전용 게이트(초록엔 미적용). 빨강 오검출이 코스를 조기종료시킬 때 켠다.
        self.declare_parameter('red_min_box_h_frac', _tc.red_min_box_h_frac)
        self.declare_parameter('red_min_conf', _tc.red_min_conf)
        # --- 팻말 방향 투표(07-15) ---
        # 실측: 모델이 같은 팻말에서 좌/우를 뒤집는다(정확도 ~60%). 게다가 뒤집힘이
        # '블록 단위'로 와서 연속 N회 디바운스로는 못 막는다(3연속쯤은 쉽게 나옴).
        # → 팻말이 보이는 동안 좌/우 신뢰도를 **누적**해 큰 쪽을 낸다(접근 구간 전체 증거).
        # 프레임 하나가 아니라 접근 전체가 한 번의 판단이 되므로 60%도 실용 수준으로 올라간다.
        # sign_vote_reset_sec 이상 팻말이 안 보이면 누적 초기화(다음 팻말과 안 섞이게).
        # ⚠ 근본 해결은 모델 재학습. 이건 노이즈 모델에서 신호를 최대한 뽑는 완화책이다.
        # [07-15 실차] True → False. 투표 누적이 **최신 판독을 이겨버렸다**:
        # 멀리서 본 RIGHT(부정확)가 점수로 쌓여, 가까이서 LEFT(conf 0.92)가 나와도 계속
        # RIGHT를 발행했다(실측 로그: dir=1 l=0.92 r=0.00 인데 sign_direction=RIGHT).
        # 이제 판단이 '팻말이 사라지기 직전의 마지막 판독'으로 기동하므로(=가장 가깝고
        # 가장 정확한 값), 과거를 섞는 투표는 오히려 그 값을 오염시킨다 → 끈다.
        # 프레임 단위 뒤집힘은 기동 중 방향 고정(decision)이 막는다.
        self.declare_parameter('sign_vote_enable', False)
        self.declare_parameter('sign_vote_reset_sec', 1.0)

        self.yolo_enable = bool(self.get_parameter('yolo_enable').value)
        self.yolo_stale_sec = float(self.get_parameter('yolo_stale_sec').value)
        self.tl_green_confirm_sec = float(self.get_parameter('tl_green_confirm_sec').value)
        self.tl_red_confirm_sec = float(self.get_parameter('tl_red_confirm_sec').value)
        self.checker_hold_sec = float(self.get_parameter('checker_hold_sec').value)
        _yolo_hz = float(self.get_parameter('yolo_max_infer_hz').value)
        self._yolo_min_interval = 1.0 / _yolo_hz if _yolo_hz > 0.0 else 0.0

        self.tl_detector = None
        if self.yolo_enable:
            try:
                _model_path = resolve_model_path(
                    self.get_parameter('yolo_model_path').value)
                _tcfg = TrafficCueConfig(
                    model_path=str(_model_path),
                    conf=float(self.get_parameter('yolo_conf').value),
                    imgsz=int(self.get_parameter('yolo_imgsz').value),
                    prefer_red=bool(self.get_parameter('tl_prefer_red').value),
                    class_green=int(self.get_parameter('tl_class_green').value),
                    class_red=int(self.get_parameter('tl_class_red').value),
                    class_left=int(self.get_parameter('tl_class_left').value),
                    class_right=int(self.get_parameter('tl_class_right').value),
                    sign_min_box_h_frac=float(
                        self.get_parameter('sign_min_box_h_frac').value),
                    sign_min_box_aspect=float(
                        self.get_parameter('sign_min_box_aspect').value),
                    red_min_box_h_frac=float(
                        self.get_parameter('red_min_box_h_frac').value),
                    red_min_conf=float(
                        self.get_parameter('red_min_conf').value),
                    cv_threads=int(self.get_parameter('yolo_cv_threads').value),
                    torch_threads=int(self.get_parameter('yolo_torch_threads').value),
                )
                self.tl_detector = TrafficLightDetector(_tcfg)
                self.get_logger().info(
                    f'YOLO 미션신호 ON: model={_model_path} conf={_tcfg.conf} '
                    f'imgsz={_tcfg.imgsz} max_hz={_yolo_hz} prefer_red={_tcfg.prefer_red} '
                    f'green_confirm={self.tl_green_confirm_sec}s stale={self.yolo_stale_sec}s')
            except Exception as e:  # noqa: BLE001 - 모델/torch 미비 시 aruco만 계속.
                self.tl_detector = None
                self.get_logger().warn(
                    f'YOLO 초기화 실패 → 신호등/체커보드 비활성(aruco만 동작): {e}')
        else:
            self.get_logger().info('yolo_enable=False → 신호등/체커보드 stub(aruco만).')

        # --- 방향 팻말(좌/우) 인지: OpenCV 색 트리거 + 팻말 YOLO(별도 모델, 학습중) ---
        # 색 트리거는 무상태·경량이라 게이트와 무관하게 매 프레임 돌린다(구간 진입 신호).
        # 방향(좌/우) YOLO는 sign_enable 게이트(SIGN_BRANCH)에서만, 모델 있을 때만 호출.
        _sd = SignConfig()
        self.declare_parameter('sign_color_lo', [int(v) for v in _sd.color_lo])
        self.declare_parameter('sign_color_hi', [int(v) for v in _sd.color_hi])
        self.declare_parameter('sign_min_px', _sd.min_px)
        self.declare_parameter('sign_roi_top_frac', _sd.roi_top_frac)
        self.declare_parameter('sign_model_path', '')       # 빈값=방향검출 비활성(색 트리거만). 모델 준비 시 경로 지정.
        self.declare_parameter('sign_conf', _sd.conf)
        self.declare_parameter('sign_imgsz', _sd.imgsz)
        self.declare_parameter('sign_class_left', _sd.class_left)
        self.declare_parameter('sign_class_right', _sd.class_right)
        self.declare_parameter('sign_stale_sec', 0.5)       # 이 시간 내 방향추론 없으면 SIGN_NONE 폴백.

        self.sign_stale_sec = float(self.get_parameter('sign_stale_sec').value)
        _sign_model = self.get_parameter('sign_model_path').value
        _sign_model_path = ''
        if _sign_model:
            try:
                _sign_model_path = str(resolve_model_path(_sign_model))
            except Exception as e:  # noqa: BLE001 - 경로 못 찾으면 색 트리거만.
                self.get_logger().warn(f'팻말 모델 경로 해석 실패(색 트리거만 동작): {e}')
        _scfg = SignConfig(
            color_lo=tuple(int(v) for v in self.get_parameter('sign_color_lo').value),
            color_hi=tuple(int(v) for v in self.get_parameter('sign_color_hi').value),
            min_px=int(self.get_parameter('sign_min_px').value),
            roi_top_frac=float(self.get_parameter('sign_roi_top_frac').value),
            model_path=_sign_model_path,
            conf=float(self.get_parameter('sign_conf').value),
            imgsz=int(self.get_parameter('sign_imgsz').value),
            class_left=int(self.get_parameter('sign_class_left').value),
            class_right=int(self.get_parameter('sign_class_right').value),
            cv_threads=int(self.get_parameter('yolo_cv_threads').value),
            torch_threads=int(self.get_parameter('yolo_torch_threads').value),
        )
        try:
            self.sign_detector = SignDetector(_scfg)
        except Exception as e:  # noqa: BLE001 - 색 트리거도 실패하면 팻말 전체 비활성.
            self.sign_detector = None
            self.get_logger().warn(f'SignDetector 초기화 실패 → 팻말 인지 비활성: {e}')
        _sign_dir_on = self.sign_detector is not None and self.sign_detector.model is not None
        self.get_logger().info(
            f'팻말 인지: 색 트리거={"ON" if self.sign_detector else "OFF"} '
            f'(min_px={_scfg.min_px}), 방향 YOLO={"ON" if _sign_dir_on else "OFF(모델 미준비)"}.')

        # YOLO 디버그 오버레이(옵션, aruco와 별도 토픽).
        self.pub_yolo_debug = None
        if self.publish_debug and self.tl_detector is not None:
            self.pub_yolo_debug = self.create_publisher(
                CompressedImage, 'perception/mission_cues/yolo/debug/compressed', 1)
        # 팻말 방향 YOLO 디버그(옵션). 모델 있을 때만.
        self.pub_sign_debug = None
        if (self.publish_debug and self.sign_detector is not None
                and self.sign_detector.model is not None):
            self.pub_sign_debug = self.create_publisher(
                CompressedImage, 'perception/mission_cues/sign/debug/compressed', 1)

        # YOLO 상시 on 모드(07-14): 페이즈별 YOLO on/off 게이팅을 제거한다. YOLO를 전
        # 구간 켜 두므로 신호등·팻말 방향을 항상 추론·발행하고, lane_mode의 게이트
        # (yolo_enable/sign_enable)는 무시한다. False로 두면 옛 페이즈 게이팅 동작으로 복귀.
        self.declare_parameter('yolo_always_on', True)
        self.yolo_always_on = bool(self.get_parameter('yolo_always_on').value)
        # YOLO 추론 게이트(판단→인지 역채널 /decision/lane_mode.yolo_enable).
        # yolo_always_on=True면 둘 다 강제 ON 고정(게이팅 무력화).
        self._yolo_gate = True
        # 팻말 방향 게이트. 상시 on이면 True(항상 방향 라우팅), 아니면 옛 기본 OFF(SIGN_BRANCH만).
        self._sign_gate = bool(self.yolo_always_on)
        lane_mode_topic = str(self.get_parameter('lane_mode_topic').value)
        self.create_subscription(
            LaneMode, lane_mode_topic, self._on_lane_mode, reliable_q)

        # YOLO 시간 상태(스로틀·초록 확정·체커 hold).
        self._yolo_last_infer: Time | None = None    ##< 마지막 추론 시각(스로틀).
        self._yolo_result_time: Time | None = None   ##< 마지막 추론 결과 시각(stale 판정).
        self._tl_raw_light = TL_NONE                  ##< 마지막 추론 프레임단위 신호.
        self._green_since: Time | None = None         ##< 초록 연속 시작 시각(확정용).
        self._red_since: Time | None = None           ##< 빨강 '구간' 시작 시각(확정용, 초록과 대칭).
        self._red_last_seen: Time | None = None       ##< 마지막 빨강 프레임 시각(tl_red_gap_sec 이어붙이기용).
        self._checker_seen_time: Time | None = None   ##< 체커보드 마지막 검출 시각(hold).
        # 팻말 방향 시간 상태(스로틀·stale).
        self._sign_last_infer: Time | None = None     ##< 마지막 방향추론 시각(스로틀).
        self._sign_result_time: Time | None = None    ##< 마지막 방향추론 결과 시각(stale).
        self._sign_dir_raw = SIGN_NONE                ##< 마지막 방향추론 프레임단위 값.
        # 팻말 방향 투표 누적(접근 구간 전체의 증거를 합쳐 좌/우 결정).
        self.sign_vote_enable = bool(self.get_parameter('sign_vote_enable').value)
        self.sign_vote_reset_sec = float(self.get_parameter('sign_vote_reset_sec').value)
        self._sign_left_score = 0.0                   ##< 누적 좌 신뢰도.
        self._sign_right_score = 0.0                  ##< 누적 우 신뢰도.
        self._sign_vote_time: Time | None = None      ##< 마지막 팻말 검출 시각(누적 리셋 판정).

        self.get_logger().info(
            f'mission_cues_node ready: sub={image_topic} → pub {cues_topic} (reliable). '
            f'ArUco dict={cfg.dictionary} ids={"ANY" if use_any else list(cfg.target_ids)} '
            f'roi_bottom={cfg.roi_bottom_frac} hold={self.hold_sec}s. '
            f'traffic_light/checkerboard=YOLO({"ON" if self.tl_detector else "OFF"}), '
            f'red_zone=stub(미구현).')

    def on_image(self, msg: CompressedImage):
        """@brief compressed 디코드 → ArUco 검출 → 홀드 → MissionCues 발행."""
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('프레임 디코드 실패', throttle_duration_sec=2.0)
            return

        res = self.detector.detect(frame, want_debug=self.publish_debug)

        # --- present 홀드(디바운스): 검출되면 즉시 on, 끊겨도 hold_sec 동안 유지 ---
        now = self.get_clock().now()
        if res.present:
            self._last_seen_time = now
            self._held_present = True
        elif self._held_present and self._last_seen_time is not None:
            elapsed = (now - self._last_seen_time).nanoseconds * 1e-9
            if elapsed >= self.hold_sec:
                self._held_present = False

        # --- YOLO 통합 미션신호(신호등+팻말): 프레임당 1회 추론, 게이트별로 라우팅 ---
        # 신호등·팻말이 한 모델(best.pt: green/left/red/right)이라 추론은 **한 번만**.
        # yolo_gate 또는 sign_gate 중 하나라도 ON이면 추론하고, 결과를 게이트별로 소비:
        #   yolo_gate ON → 신호등(초록 연속 확정 / 빨강) 갱신
        #   sign_gate ON → 방향팻말(좌/우) 갱신
        # 둘 다 OFF(순수 주행)면 skip → CPU/FPS 확보. stale 지나면 _resolve_*가 폴백.
        # aruco/차선 검출은 게이트와 무관하게 계속 동작.
        if self.tl_detector is not None and (self._yolo_gate or self._sign_gate):
            due = (self._yolo_last_infer is None or
                   (now - self._yolo_last_infer).nanoseconds * 1e-9
                   >= self._yolo_min_interval)
            if due:
                self._yolo_last_infer = now
                want_dbg = (self.pub_yolo_debug is not None or
                            self.pub_sign_debug is not None)
                det = self.tl_detector.detect(frame, want_debug=want_dbg)
                # 신호등: yolo_gate ON 일 때만 소비(분기/주행 중 가짜 신호 차단).
                if self._yolo_gate:
                    self._yolo_result_time = now
                    self._tl_raw_light = det.light
                    # 초록이면 연속 시작시각 유지, 아니면 확정 타이머 리셋.
                    self._green_since = (
                        (self._green_since or now) if det.light == TL_GREEN else None)
                    # 빨강 '구간' 추적. 초록과 달리 한 프레임만 끊겨도 리셋하지 않는다:
                    # 모델이 같은 하우징의 빨강↔초록을 깜빡여(한 프레임 초록이 conf로 이김)
                    # 연속 확정이 매번 리셋 → 마지막 빨강을 놓쳤다. tl_red_gap_sec 이내에
                    # 빨강이 다시 보이면 같은 구간으로 이어 붙이고, 그보다 오래 끊겨야 구간 종료.
                    if det.light == TL_RED:
                        if self._red_since is None:
                            self._red_since = now          # 새 빨강 구간 시작.
                        self._red_last_seen = now
                    elif (self._red_last_seen is not None
                          and (now - self._red_last_seen).nanoseconds * 1e-9
                          > self.tl_red_gap_sec):
                        self._red_since = None             # gap 초과로 끊김 → 구간 종료.
                        self._red_last_seen = None
                # 방향팻말: sign_gate ON(SIGN_BRANCH) 일 때만 소비.
                if self._sign_gate:
                    self._sign_result_time = now
                    self._sign_dir_raw = det.direction
                    # 방향 투표 누적: 게이트를 통과한 좌/우 신뢰도를 접근 구간 내내 더한다.
                    # 오래 못 봤으면(=다른 팻말/새 접근) 먼저 초기화해 증거가 안 섞이게 한다.
                    if self.sign_vote_enable and (det.left_conf > 0.0
                                                  or det.right_conf > 0.0):
                        if (self._sign_vote_time is None
                                or (now - self._sign_vote_time).nanoseconds * 1e-9
                                > self.sign_vote_reset_sec):
                            self._sign_left_score = 0.0
                            self._sign_right_score = 0.0
                        self._sign_left_score += float(det.left_conf)
                        self._sign_right_score += float(det.right_conf)
                        self._sign_vote_time = now
                # 디버그 오버레이(통합 1장) → 있는 패널에 발행.
                if det.debug_image is not None:
                    if self.pub_yolo_debug is not None:
                        self._publish_jpeg(self.pub_yolo_debug, det.debug_image, msg.header.stamp)
                    if self.pub_sign_debug is not None:
                        self._publish_jpeg(self.pub_sign_debug, det.debug_image, msg.header.stamp)
                self.get_logger().info(
                    f'yolo light={det.light} g={det.green_conf:.2f} r={det.red_conf:.2f} '
                    f'dir={det.direction} l={det.left_conf:.2f} r={det.right_conf:.2f} '
                    f'boxes={det.num_boxes} '
                    # 팻말 게이트 튜닝: h=박스높이비(거리), asp=w/h(각도). GATED=임계 미달로 무시됨.
                    f'sign_h={det.sign_h_frac:.3f} asp={det.sign_aspect:.2f}'
                    f'{" [GATED:멀거나 비스듬→무시]" if det.sign_gated else ""}'
                    # 빨강 게이트 튜닝: red_h=빨강 박스높이비(거리).
                    f' red_h={det.red_h_frac:.3f}'
                    f'{" [RED-GATED]" if det.red_gated else ""}'
                    # 방향 투표 누적점수: 접근 구간 전체 증거. 큰 쪽이 최종 방향.
                    f' vote(L={self._sign_left_score:.1f} R={self._sign_right_score:.1f})',
                    throttle_duration_sec=1.0)

        # --- 방향 팻말 색 트리거(항상, 경량·무상태): 구간 진입(sign_present)만 판정 ---
        # 방향(좌/우)은 위 통합 추론이 담당하므로 여기선 색 트리거만 돌린다.
        sign_present = False
        if self.sign_detector is not None:
            col = self.sign_detector.detect_color(frame)
            sign_present = bool(col.present)

        tl_state, checker_det = self._resolve_cues(now)
        sign_dir = self._resolve_sign(now)

        # --- MissionCues 발행(aruco/신호등/체커보드/팻말 실제값, red_zone stub) ---
        cue = MissionCues()
        cue.header.stamp = msg.header.stamp
        cue.header.frame_id = self.base_frame
        cue.traffic_light = tl_state                 # YOLO 신호등(초록 확정 후 GREEN).
        cue.checkerboard_detected = checker_det      # YOLO 체커보드(hold). 종료판정 폐기(07-14), 필드만.
        cue.red_zone_detected = False                # stub(폐기).
        cue.aruco_present = bool(self._held_present)
        cue.sign_detected = sign_present             # OpenCV 팻말색 트리거(구간 진입).
        cue.sign_direction = sign_dir                # 팻말 YOLO 좌/우(게이트 ON·fresh일 때만).
        self.pub_cues.publish(cue)

        if self.pub_debug is not None and res.debug_image is not None:
            self._publish_jpeg(self.pub_debug, res.debug_image, msg.header.stamp)

        self._frames += 1
        if res.present or self._frames % 30 == 0:
            self.get_logger().info(
                f'aruco raw={res.present} ids={res.ids} → present(held)={self._held_present}',
                throttle_duration_sec=0.5)

    def _on_lane_mode(self, msg: LaneMode):
        """@brief 판단 역채널 수신 → 신호등/팻말 YOLO 게이트 갱신(전이 시 로그)."""
        # YOLO 상시 on 모드: 외부 게이팅 무시(게이트는 __init__에서 강제 ON 고정).
        # 판단이 같은 LaneMode로 turn_bias를 보내도 YOLO가 꺼지지 않게 한다.
        if self.yolo_always_on:
            return
        gate = bool(msg.yolo_enable)
        if gate != self._yolo_gate:
            self.get_logger().info(
                f'YOLO gate {"ON" if gate else "OFF"} (from mission /decision/lane_mode)')
            # OFF 전이 시 초록 확정 타이머를 접어 재점화 후 stale 신호가 안 남게 한다.
            if not gate:
                self._green_since = None
                self._red_since = None
                self._red_last_seen = None   # 구간 추적도 같이 접어야 재점화 후 즉시 확정 안 됨.
        self._yolo_gate = gate

        # 팻말 YOLO 게이트(sign_enable). 발행자가 필드를 안 실으면 기본 False로 읽힌다.
        sign_gate = bool(getattr(msg, 'sign_enable', False))
        if sign_gate != self._sign_gate:
            self.get_logger().info(
                f'SIGN gate {"ON" if sign_gate else "OFF"} (from mission /decision/lane_mode)')
        self._sign_gate = sign_gate

    def _resolve_cues(self, now):
        """@brief 시간 상태로 발행할 traffic_light + checkerboard 계산.

        @details 추론이 stale(오래됨)하거나 없으면 NONE/false로 폴백(끊긴 프레임에 붙들려
        가짜 신호 유지 방지). 초록은 tl_green_confirm_sec 동안 **연속**돼야 TL_GREEN을 낸다
        (한 프레임 가짜 초록 오출발 방지) — 확정 전엔 NONE(대기).
        빨강은 tl_red_confirm_sec 동안 지속돼야 TL_RED. 단 '지속'의 판정에 raw 신호를 쓰지
        않고 _red_last_seen(gap 이내 빨강을 봤는가)을 쓴다 — 깜빡임으로 raw가 한 프레임
        GREEN이 돼도 구간이 유지돼야 하기 때문(raw로 게이트하면 gap 이어붙이기가 무의미해짐).
        체커보드는 hold로 잠깐 놓쳐도 유지. @return (traffic_light, checkerboard_detected).
        """
        if self.tl_detector is None or self._yolo_result_time is None:
            return MissionCues.TL_NONE, False
        fresh = (now - self._yolo_result_time).nanoseconds * 1e-9 <= self.yolo_stale_sec
        if not fresh:
            return MissionCues.TL_NONE, False

        # 빨강을 gap 이내에 봤는가(깜빡임 허용). raw가 지금 초록이어도 구간은 살아 있다.
        red_recent = (self._red_last_seen is not None and
                      (now - self._red_last_seen).nanoseconds * 1e-9 <= self.tl_red_gap_sec)
        # 확정은 '첫 빨강~마지막 빨강'의 실제 지속(span)으로 잰다. now 기준으로 재면 안 된다:
        # gap(0.5s) > confirm(0.2s)이라 **단발 오검출 한 프레임**도 gap이 만료되기 전까지
        # 타이머가 혼자 흘러 confirm을 채워 코스를 끝내버린다(span이면 단발은 0이라 안전).
        red_span = (0.0 if (self._red_since is None or self._red_last_seen is None)
                    else (self._red_last_seen - self._red_since).nanoseconds * 1e-9)
        if red_recent and red_span >= self.tl_red_confirm_sec:
            light = MissionCues.TL_RED
        elif (self._tl_raw_light == TL_GREEN and self._green_since is not None and
              (now - self._green_since).nanoseconds * 1e-9 >= self.tl_green_confirm_sec):
            light = MissionCues.TL_GREEN
        else:
            light = MissionCues.TL_NONE   # 초록 확정 전/미검출 → 대기.

        checker = (self._checker_seen_time is not None and
                   (now - self._checker_seen_time).nanoseconds * 1e-9 <= self.checker_hold_sec)
        return light, checker

    def _resolve_sign(self, now):
        """@brief 발행할 sign_direction 계산(게이트 OFF/미검출/stale이면 SIGN_NONE).

        @details 팻말 방향은 SIGN_BRANCH(게이트 ON)에서만 유효. 추론이 stale하면 NONE으로
        폴백(멈춘 프레임 신호에 붙들리지 않음). 좌/우 확정(안정화)은 판단(mission)이 첫
        non-NONE을 래치해 담당한다 — 여기선 프레임 단위 최신값만 낸다. @return sign_direction.
        """
        if (not self._sign_gate or self._sign_result_time is None):
            return SIGN_NONE
        fresh = (now - self._sign_result_time).nanoseconds * 1e-9 <= self.sign_stale_sec
        if not fresh:
            return SIGN_NONE
        # 투표: 접근 구간 내내 누적한 좌/우 신뢰도 합에서 큰 쪽. 프레임 단위 뒤집힘
        # (모델 정확도 ~60%, 블록성 오류)에 휘둘리지 않는다.
        if self.sign_vote_enable and self._sign_vote_time is not None:
            stale_vote = ((now - self._sign_vote_time).nanoseconds * 1e-9
                          > self.sign_stale_sec)
            if not stale_vote and (self._sign_left_score > 0.0
                                   or self._sign_right_score > 0.0):
                return (SIGN_RIGHT if self._sign_right_score >= self._sign_left_score
                        else SIGN_LEFT)
            return SIGN_NONE
        return self._sign_dir_raw

    def _publish_jpeg(self, pub, img, stamp):
        ok, enc = cv2.imencode(
            '.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), self.debug_quality])
        if not ok:
            return
        out = CompressedImage()
        out.header.stamp = stamp
        out.header.frame_id = self.base_frame
        out.format = 'jpeg'
        out.data = enc.tobytes()
        pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCuesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
