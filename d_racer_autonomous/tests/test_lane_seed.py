"""@file test_lane_seed.py
@brief lost 재획득 시 base fallback(차선 미검출 → 화면 25%/75%) 테스트."""
import numpy as np

from core.perception.lane_detect import LaneCalib, LaneDetector


def _detector():
    det = LaneDetector(LaneCalib(seed_min_fill=0.05))
    det._last_leftx = 0.0    # 직전값을 일부러 엉뚱하게(맨왼쪽) 둔다.
    det._last_rightx = 0.0
    return det


def _hist(w, entries):
    """열별 에지합 히스토그램 생성. entries=[(col, mass), ...]."""
    hist = np.zeros(w, dtype=np.float64)
    for col, mass in entries:
        hist[col] = mass
    return hist


def test_no_lane_defaults_to_quartiles():
    """양쪽 다 peak 없음 → base가 화면 25%/75% 기본위치."""
    det, w, h = _detector(), 320, 160
    leftx, rightx = det._reacquire_base(_hist(w, []), w, h)
    assert leftx == int(w * 0.25)
    assert rightx == int(w * 0.75)


def test_weak_peak_below_threshold_is_ignored():
    """임계 미만 노이즈 peak은 무시하고 기본위치 사용."""
    det, w, h = _detector(), 320, 160
    # min_mass = 255 * (160 - 96) * 0.05 = 816. 그 아래는 노이즈.
    hist = _hist(w, [(40, 500.0), (250, 500.0)])
    leftx, rightx = det._reacquire_base(hist, w, h)
    assert leftx == int(w * 0.25)
    assert rightx == int(w * 0.75)


def test_strong_peak_is_adopted():
    """임계 이상 peak은 argmax로 채택(직전값과 블렌딩)."""
    det, w, h = _detector(), 320, 160
    b = det.calib.seed_reacquire_blend
    prev_l, prev_r = det._last_leftx, det._last_rightx
    hist = _hist(w, [(40, 5000.0), (250, 5000.0)])   # 충분한 에지량
    leftx, rightx = det._reacquire_base(hist, w, h)
    # _last_* 과 seed_reacquire_blend 블렌딩 → config 값 추종(blend 튜닝돼도 안 깨짐).
    assert leftx == int(prev_l * (1 - b) + 40 * b)
    assert rightx == int(prev_r * (1 - b) + 250 * b)


def test_one_side_missing_only_that_side_defaults():
    """오른쪽만 보이면 왼쪽만 기본위치, 오른쪽은 peak 채택."""
    det, w, h = _detector(), 320, 160
    b = det.calib.seed_reacquire_blend
    prev_r = det._last_rightx
    hist = _hist(w, [(250, 5000.0)])                 # 오른쪽만 강한 peak
    leftx, rightx = det._reacquire_base(hist, w, h)
    assert leftx == int(w * 0.25)                    # 왼쪽 미검출 → 기본 25%
    assert rightx == int(prev_r * (1 - b) + 250 * b)  # 오른쪽 채택(config blend 추종)


def test_collapse_guard_recovers_center_and_separates_seed():
    """좌·우 탐색창이 한 실선에 달라붙는 붕괴 → 중심선 정상화 + seed 분리(복귀).

    회귀 방지: 가드 없으면 중심선이 그 선 위(~100)로 튀고 두 seed가 겹쳐(gap≈0)
    lock인 채 영구 이탈한다. 가드가 단일차선 강등으로 중심을 차선중심(~160)에
    되돌리고 seed를 lane_w만큼 벌려 다음 프레임에 복귀시킨다.
    """
    w, h = 320, 160
    edges = np.zeros((h, w), np.uint8)
    edges[:, 98:102] = 255                            # x≈100 단일 실선(오른쪽 끊김)

    det = LaneDetector(LaneCalib(lane_width_px=120.0))
    det._last_leftx, det._last_rightx = 90.0, 130.0   # 두 seed가 x=100 근처로 몰린 직전상태
    det._last_conf = 1.0                              # lock
    det._lane_width_px = 120.0

    center, _ = det._sliding_window(edges, None)
    assert abs(center[0][0] - 160.0) < 15.0           # 차선중심 복원(그 선 위 ~100 아님)
    assert (det._last_rightx - det._last_leftx) > 90.0  # seed 분리(붕괴면 ≈0)
