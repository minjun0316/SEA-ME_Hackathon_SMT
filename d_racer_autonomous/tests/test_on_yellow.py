"""@file test_on_yellow.py
@brief 노랑 지름길 래치(on_yellow) hysteresis 단위 테스트.

계약(LaneStatus.on_yellow): 인지가 노랑 우세를 '안정 검출'하면 ON, 점선 갭엔 유지.
LaneDetector._update_on_yellow(yellow_px, white_px)의 enter/exit 비대칭 래치를 검증한다.
카메라 없이 순수 로직만 확인(프레임 합성 불필요).
"""
from core.perception.lane_detect import LaneCalib, LaneDetector


def _make(enter=3, leave=5, min_px=100, dom=1.2):
    """작은 enter/exit로 래치를 만든다(테스트 가독성)."""
    calib = LaneCalib(
        on_yellow_min_px=min_px,
        on_yellow_dom_ratio=dom,
        on_yellow_enter_frames=enter,
        on_yellow_exit_frames=leave,
    )
    return LaneDetector(calib)


def _feed(det, yellow_px, white_px, n):
    """같은 (yellow, white)를 n프레임 먹이고 마지막 래치 상태 반환."""
    last = det._on_yellow
    for _ in range(n):
        last = det._update_on_yellow(yellow_px, white_px)
    return last


def test_latches_on_after_enter_frames():
    """우세 프레임이 enter_frames 연속돼야 ON(그 전엔 OFF)."""
    det = _make(enter=3)
    assert _feed(det, 500, 100, 2) is False      # 2프레임: 아직 OFF
    assert det._update_on_yellow(500, 100) is True  # 3프레임째 ON


def test_holds_through_short_gap():
    """ON 이후 점선 갭(비우세 < exit_frames)은 래치 유지."""
    det = _make(enter=3, leave=5)
    _feed(det, 500, 100, 3)                      # ON
    assert det._on_yellow is True
    assert _feed(det, 0, 100, 4) is True         # 갭 4(<5) → 유지


def test_latches_off_after_exit_frames():
    """비우세가 exit_frames 연속되면 OFF."""
    det = _make(enter=3, leave=5)
    _feed(det, 500, 100, 3)                      # ON
    assert _feed(det, 0, 100, 5) is False        # 비우세 5 연속 → OFF


def test_gap_then_recovery_resets_exit_counter():
    """갭 중간 우세 1프레임이 OFF 카운터를 리셋 → 다시 갭 나도 유지."""
    det = _make(enter=3, leave=5)
    _feed(det, 500, 100, 3)                      # ON
    _feed(det, 0, 100, 4)                        # 갭 4(유지)
    _feed(det, 500, 100, 1)                      # 노랑 복귀 → notlead_run 리셋
    assert _feed(det, 0, 100, 4) is True         # 다시 갭 4 → 연속 끊겨 유지


def test_white_dominant_does_not_latch():
    """노랑 픽셀 충분해도 흰색이 더 많으면(우세 아님) 래치 안 걸림."""
    det = _make(enter=3, dom=1.2)
    assert _feed(det, 500, 1000, 10) is False    # 500 < 1000×1.2 → 비우세


def test_below_min_px_does_not_latch():
    """노랑이 흰색보다 많아도 절대 하한 미달이면 우세 아님."""
    det = _make(enter=3, min_px=100)
    assert _feed(det, 50, 0, 10) is False        # 50 < min_px 100
