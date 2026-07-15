"""@file lane_detect.py
@brief ROS-free 차선 인지 순수 로직(OpenCV). `lane_only_detect.cpp` 알고리즘의 파이썬 포팅.

@details
계약(docs/interfaces.md §4.2/4.3)에 맞춰 결과를 낸다:
- `lane_path`: 차선 중심선 점열, **base_link 미터 좌표**(+x 전방, +y 좌측), near→far.
- `LaneResult`(= racer_msgs/LaneStatus 미러): lane_detected/confidence/num_points/
  lateral_offset[m,+좌]/heading_error[rad]/stop_line/stop_line_dist[m] + 노랑·흰 검출.

핵심 파이프라인(cpp와 동일): BEV 원근변환 → HLS 노랑/흰 마스크 → (노랑 우세면 노랑만) →
Canny 에지 → 슬라이딩 윈도우로 좌/우 차선 추적 → 창별 중심점으로 centerline(px) →
**픽셀→미터 변환**(base_link) → offset/heading/정지선거리 산출.

@note 픽셀→미터 스케일(`m_per_px_*`, `x_near_m`)은 **캘리브 전 잠정값**이다. BEV가
      실제 지면 미터로 캘리된 게 아니므로 트랙에서 실측·튜닝해야 정확하다(계약대로
      lane_path는 미터여야 controller Pure Pursuit가 맞다).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class LaneCalib:
    """@brief 차선 인지 캘리브/임계값(전부 파라미터, 하드코딩 금지)."""

    # --- BEV 원근변환 src(이미지 크기 대비 비율). dst는 전체 사각형 ---
    bev_top_y: float = 0.4      ##< 상단 라인 y 비율(작을수록 멀리까지)
    bev_top_x: float = 0.2      ##< 상단 좌/우 x 비율(좌=x, 우=1-x)
    # 체커보드 IPM으로 실측한 BEV 호모그래피(이미지→BEV, 3x3 행렬을 행우선 9값).
    # 지정되면 4점(bev_top_*) 대신 이 행렬을 그대로 쓴다. None이면 기존 4점 폴백.
    bev_matrix: Optional[Tuple[float, ...]] = None

    # --- HLS 색 임계 (H, L, S) ---
    # 흰 HLS 임계(H,L,S). 2026-07-14 인지팀 실측값(calibrate_hls)으로 갱신.
    # OpenCV BGR2HLS 규약: H 0~179, L/S 0~255. 실차 lane.yaml과 동기화한다(yaml이 최종
    # 소스지만 누락 키가 stale 기본값으로 새지 않도록 일치시킴). 재측정 시 양쪽 갱신.
    # 노랑(yellow_lo/hi)은 차선추종에선 폐기됐고 이제 _stopline(정지선색)만 참조한다.
    yellow_lo: Tuple[int, int, int] = (0, 146, 129)  ##< 정지선(stopline_color=yellow) 전용.
    yellow_hi: Tuple[int, int, int] = (153, 225, 255)  ##< 〃
    white_lo: Tuple[int, int, int] = (0, 213, 0)     ##< white_adaptive=False 경로가 사용(하한)
    white_hi: Tuple[int, int, int] = (172, 255, 255)  ##< 〃 상한
    # 흰색 마스크 방식. 기본 False = 원본 C++ 고정 임계(white_lo/hi = inRange((0,200,0),
    # (180,255,70)))를 그대로 사용 — 밝은 흰색은 L≥200이면 무조건 통과(원본 검증된 로직).
    # True = 적응형(L 하한 = k×L상위퍼센타일, 조명 불변). 적응형은 회색오검·저조도 깜빡임을
    # 없애지만, 프레임에 흰 선보다 더 밝은 것(tops글레어)이 있으면 기준이 올라가 '밝은 흰색을
    # 놓치는' 부작용이 있어 기본은 원본 고정으로 둔다. 조명 변동이 심하면 True로 실험.
    white_adaptive: bool = False        ##< False=원본 C++ 고정(white_lo/hi), True=적응형
    # L 하한80 k × (L 상위 white_pct 퍼센타일). 낮출수록 완화(더 어두운 흰색까지 통과).
    # 0.80→0.72(07-10): 흰 선보다 더 밝은 것(글레어/반사)이 프레임에 있으면 p99가 그쪽으로
    # 올라가 L_lo가 높아지고 정작 흰 선이 그 밑으로 빠져 '밝은 흰색이 안 잡히던' 문제 완화.
    # k=흰 선이 최고밝기의 이 비율 이상이면 통과 → 0.72면 72%까지 허용(글레어 여유↑).
    # 여전히 밝은 흰색을 놓치면 0.68→0.65로, 회색 바닥까지 잡혀 노이즈 생기면 0.75~0.78로.
    white_adaptive_k: float = 0.65
    white_pct: float = 99.0             ##< 밝기 기준 퍼센타일(흰 선=최고밝기 추종)
    white_l_floor: int = 90             ##< L 하한 최소(칠흑 프레임서 노이즈 폭주 방지)
    white_l_cap: int = 250              ##< L 하한 최대(과도한 상승 방지)
    white_s_hi: int = 75               ##< 흰색 최대 채도 S(무채색만 통과, 유채색 배제)
    white_pixel_threshold: int = 200   ##< 흰 픽셀 이 이상이면 white_detected=True(진단용 하한).

    # --- 에지(Canny) 입력/임계 ---
    # 원래는 (마스킹 BGR→gray→blur→Canny). 마스크가 이미 이진 차선영역이라, blur한
    # gray 대신 '정리한 마스크'에 Canny를 걸면 차선 윤곽이 연속·깨끗해진다(끊김↓).
    edge_from_mask: bool = True        ##< True=정리한 lane_mask에 Canny(권장), False=옛 gray/blur
    edge_close_ksize: int = 3          ##< MORPH_CLOSE/dilate 커널 크기(작은 구멍 메우기)
    edge_dilate_iter: int = 1          ##< dilate 반복(차선 두껍게→창별 minpix 안정). 0=off
    canny_lo: int = 50                 ##< Canny 하한(옛 50→30, 흰선 민감도↑). 노이즈면 ↑
    canny_hi: int = 150                 ##< Canny 상한(옛 150→90). lo의 ~3배 유지 권장

    # --- 슬라이딩 윈도우 ---
    nwindows: int = 9
    margin: int = 30
    minpix: int = 5
    # seed(탐색 시작 x): 직전 프레임 confidence가 이 값 이상이면 '추종 중(lock)'으로
    # 보고 직전 피팅 near-x를 그대로 시드로 쓴다(search-around-poly) → 곡선 연속추종.
    # 미만이면 'lost'로 보고 하단 히스토그램으로 재획득. midpoint 고정분할 히스토그램은
    # 곡선에서 양 차선이 한쪽으로 쏠릴 때 좌우를 뒤바꾸므로 lost일 때만 최소로 쓴다.
    seed_lock_conf: float = 0.40
    # lost 재획득 시 하단 히스토그램 peak을 '실제 차선'으로 인정할 최소 에지 비율.
    # peak 열의 에지량이 (스캔행수×255×이 값) 미만이면 그쪽 차선이 안 보이는 것으로
    # 보고 base를 화면 25%/75% 기본위치로 고정한다(argmax가 0/노이즈를 잡는 것 방지).
    seed_min_fill: float = 0.05
    # lost 재획득 시 argmax peak를 직전 seed와 섞는 히스토그램 가중치(0~1).
    # base = prev*(1-b) + peak*b. 0.5=반반(기존 하드코딩). 1.0=peak만, 0.0=prev만.
    seed_reacquire_blend: float = 0.8
    # lock(추종 중) 상태에서도 매 프레임 히스토그램 peak를 seed에 소폭 섞는 가중치(0~1).
    # 0.0=off(순수 search-around-poly, 기존 동작). 0.2면 seed=prev*0.8 + peak*0.2.
    # 단순 EMA가 아니라: prev seed ±seed_hist_band_px 밴드 안에서만 peak를 찾고 
    # (윈도우 탐색 → 반대 차선/노이즈로의 점프 배제), 그 밴드 에지량이 seed_min_fill
    # 이상일 때만 섞는다(garbage 프레임은 유지). 커브에서 lock seed가 뒤처지는 지연을
    # 줄이는 용도. 켜면 지연↓·추종성↑, 과하면 노이즈로 seed가 떨릴 수 있어 0.1~0.3 권장.
    seed_lock_hist_blend: float = 0.3
    seed_hist_band_px: float = 60.0     ##< lock 블렌딩 시 prev 주변 peak 탐색 밴드(±px)
    # 한쪽 차선이 화면 밖으로 나갔을 때(커브) 복원용 차선폭[BEV px]. 두 선이 다
    # 보이는 프레임에서 자동 학습하며, 이 값은 학습 전/한번도 못 본 경우의 초기값.
    lane_width_px: float = 180.0
    # 양쪽 차선이 다 보이는 프레임에서 관측 간격으로 차선폭을 EMA 학습(True)할지, 끄고
    # lane_width_px 고정값만 쓸지(False). 고정 폭이 실측과 잘 맞거나, 학습이 노이즈로
    # 표류해 복원 폭이 흔들릴 때 False로 잠근다(한쪽 소실 복원·붕괴판정 모두 고정폭 사용).
    lane_width_learn: bool = True
    # 곡선에서 안쪽 차선이 프레임 밖으로 나가면 좌/우 두 탐색창이 남은 바깥선 하나에
    # 모두 달라붙는다. 두 피팅선 간격이 이 비율×lane_w보다 좁으면 '같은 선을 중복
    # 검출'로 보고 단일 차선으로 강등한다(→ 곡선방향 기반 안쪽 복원).
    lane_collapse_frac: float = 0.5

    # --- 단일 기준선 고정 추종(single_anchor) — 오른쪽 한 선만 추적 ---
    # dual 두 창의 붕괴·좌우 정체성 문제를 원천 차단하는 대안 모드. 기준선(anchor_side)
    # 하나만 추적하고 중심선 = 기준선 ∓ lane_w/2. 소실 시 anchor_hold_frames 동안 직전
    # 피팅 유지(coast)해 점선 갭에서 반대선으로 안 튐. 바이어스가 W/2라 lane_width_px 실측 +
    # lane_width_learn=false 권장. 기본 off(dual 유지), lane.yaml single_anchor=true로 A/B.
    single_anchor: bool = False        ##< True=단일 기준선 고정 추종(dual 우회).
    anchor_side: str = 'right'         ##< 고정 기준 경계선: 'right'(기본) | 'left'.
    anchor_hold_frames: int = 10       ##< 기준선 소실 시 직전 피팅 유지 최대 프레임(점선 갭).

    # --- 적응형 anchor(3-state 하이브리드): 직선/양선=dual, 커브=바깥선 single ---
    # 기하: 커브에서는 안쪽 선이 BEV 화면 옆으로 먼저 빠져나가고 바깥(커브 반대쪽) 선이
    # 프레임에 남는다 → 오른쪽 커브=왼선 anchor, 왼쪽 커브=오른선 anchor. 직선·양선
    # 뚜렷하면 dual(양선 중앙). 커브에선 안쪽(노이즈·소실) 선을 아예 무시해 dual 붕괴/
    # 조기치우침을 원천 차단. single_anchor(고정)와 배타 — single_anchor=True면 이건 무시.
    adaptive_anchor: bool = False      ##< True=커브방향 따라 dual↔단일(바깥선) 자동 전환.
    both_enter_frames: int = 3         ##< 단일→dual 승격: 양선 뚜렷+직선 이만큼 연속 프레임.
    both_exit_frames: int = 3          ##< dual→단일(또는 좌↔우) 전환: 커브 이만큼 연속 프레임.
    curve_dx_deadband_px: float = 18.0  ##< |near-x−far-x| 이 값 초과면 '커브'로 판정(직선 격리).
    # 팻말 강제 anchor의 커브 게이트(07-15): |curve_dx|가 이 값을 넘으면(=커브 중) 팻말
    # 지시를 무시하고 기하학(adaptive) 추종을 유지한다. S자 끝은 팻말과 거리가 가까워
    # 거리 게이트(sign_min_box_h_frac)로는 안 갈렸다 → '커브냐 직선이냐'라는 다른 축으로 분리.
    # 직선이 되면 그때 팻말이 먹는다. 0=게이트 끔(커브에서도 팻말 강제 허용).
    sign_force_max_curve_px: float = 18.0  ##< 이 값 초과 커브면 팻말 강제 무시(기하학 우선).
    # S자 변곡점 대책(07-15): 좌↔우가 바뀌는 순간 curve_dx가 0을 지나 '직선'으로 보여
    # 게이트가 열리고 팻말이 튀어들어왔다(프레임 하나로는 변곡점을 커브라 판정 불가 —
    # 피팅이 2차라 S자를 표현 못 함). → '직선이 이만큼 연속'돼야 팻말을 허용한다.
    # 변곡점의 찰나 직선은 이 수를 못 채워 막히고, 진짜 직선 접근로는 채워서 통과한다.
    sign_force_straight_frames: int = 15  ##< 팻말 강제 허용에 필요한 연속 직선 프레임(~0.75s@20fps).
    # [07-16 실차] 위 두 게이트를 **노이즈가 무력화**했다. curve_dx = 피팅선의 far−near라
    # BEV 원거리 끝점 흔들림을 그대로 먹어, 실측 로그에서 50ms 간격으로 +45 → −49로 부호가
    # 뒤집혔다(차가 그 속도로 커브를 바꿀 리 없다 = 순수 노이즈). |노이즈| 최대 62 > 임계 18
    # 이라 연속 직선 카운터가 계속 0으로 리셋 → 팻말이 **한 프레임만** 발동하고 무너졌다.
    # 그 로그의 중앙값은 +3.5 = 차는 실제로 직선 위에 있었다. → 게이트 판정 전에 중앙값
    # 필터를 먹인다. 임계를 올리는 건 답이 아니다(65+가 필요 = 게이트를 없애는 것과 같음).
    # 창 크기는 그 로그를 재생해 정했다(scratchpad/verify_median_gate.py): 노이즈 std가
    # ~30px이라 5로는 롤링 중앙값이 아직 출렁여 4/22프레임만 발동했다. **7이면 12/22가
    # 연속(0.6s@20fps)으로 발동**하고, 진짜 커브(dx 30~52 연속)는 그대로 0/22로 막힌다.
    # 임계 18은 손대지 않는다(25로 올리면 dx≈20인 완만한 진짜 커브가 새 나간다).
    # 지연은 (N-1)/2 = 3프레임(0.15s@20fps)뿐이라 판독/발동 분리는 유지된다.
    # 1 또는 0 = 필터 끔(옛 동작). 커브를 놓치면 ↓(5), 노이즈가 여전히 새면 ↑(9~11).
    sign_curve_median_frames: int = 7     ##< curve_dx 중앙값 필터 창(프레임). 1=끔.
    # 팻말 적용 방식(07-15). 실트랙은 팻말로 이어지는 접근로가 '커브'라, anchor 교체
    # 방식은 커브 기하학을 파괴해 추종이 깨지고(강제하면) / 직선 게이트에 막혀 영영
    # 발동을 못 해(안 하면) 팻말을 들이받았다 — 커브 접근로에선 양립 불가.
    #   "offset"(기본) = 기하학이 만든 중심선을 팻말 반대쪽으로 평행이동만 한다.
    #                    차선 추종 로직을 전혀 안 건드려 커브에서도 안전. 커브/직선
    #                    게이트(sign_force_*) 불필요.
    #   "anchor"       = 옛 방식(지시된 쪽 끝차선으로 anchor 교체). 직선 구간 전용.
    sign_apply: str = "offset"           ##< "offset" | "anchor".
    perp_offset: bool = True           ##< 단일 anchor 시 W/2 오프셋을 법선(수직)으로(곡선 치우침 보정).
    perp_max_slope: float = 1.5        ##< 법선 배율 √(1+slope²)의 slope(dx/dy) 절댓값 상한(폭주 방지).

    # 중심선 노이즈 완화: 경로점을 다항식(y~x)으로 피팅해 매끄럽게. 슬라이딩윈도우
    # 창별 흔들림이 조향 휘청임으로 이어지는 걸 방지(직선·커브 공통). order 2면 곡선까지.
    path_smooth: bool = True
    path_smooth_order: int = 2

    # --- 정지선(행별 색상 커버리지 밴드) ---
    # BEV 하단 ROI에서 "가로로 길게 정지선색"인 행이 여러 개 모이면 정지선. 차선은
    # BEV서 세로라 행 커버리지가 낮고, 정지선(가로띠)만 높다 → 차선/노이즈와 잘 분리.
    # 색 마스크는 stopline_color로 선택(실트랙 정지선=노랑). 차선도 노랑이지만 세로라
    # 행 커버리지가 낮아 정지선(가로띠)과 구분된다. @see _stopline
    stopline_color: str = "yellow"       ##< 정지선 색: yellow|white|both. 실트랙 정지선=노랑.
    stopline_roi_h: int = 90             ##< 하단 ROI 높이[px](정지선 탐색 구간).
    stopline_row_coverage: float = 0.45  ##< 한 행이 이 비율 이상 정지선색이면 후보 행(하한).
    stopline_max_width_m: float = 40.0    ##< 가로폭 상한[m]: 한 행의 '연속' 정지선색 폭이 이 값 초과면 가로로 너무 긺→후보 제외(m_per_px_lateral로 px 환산). 0=상한없음. 실측 정지선폭=0.35.
    stopline_min_rows: int = 6           ##< 후보 행이 이만큼 이상이면 정지선 검출.
    stopline_len_threshold: float = 150.0  ##< hough 방식: 수평 선분 길이합 임계(넘으면 정지선). 작년 검증값 150.
    # --- 정지선 검출 방식 스위치(07-14, 작년 Hough 재이식) ---
    # "coverage"=행별 색 커버리지(조명/점선 강건, 07-14 도입). "hough"=작년에 실제로
    # 잘 잡던 방식(정지선색 마스크→Canny→HoughLinesP→'수평' 선분 길이합 > len_threshold).
    # 실트랙서 coverage가 정지선을 못 잡으면 hough로 전환(색은 stopline_color 그대로).
    stopline_method: str = "coverage"     ##< "coverage"|"hough".
    stopline_canny_lo: int = 100          ##< hough: 마스크 Canny 하한.
    stopline_canny_hi: int = 200          ##< hough: 마스크 Canny 상한.
    stopline_hough_thresh: int = 40       ##< hough: HoughLinesP 누적 임계.
    stopline_hough_min_len: int = 40      ##< hough: 최소 선분 길이[px].
    stopline_hough_max_gap: int = 5       ##< hough: 선분 내 최대 갭[px].
    stopline_angle_tol_deg: float = 10.0  ##< hough: 수평 판정 허용각[deg](|angle|<tol 또는 >180-tol). 세로 차선 배제.

    # --- 픽셀→미터(BEV 기준). 캘리브 전 잠정값 → 트랙 튜닝 ---
    m_per_px_forward: float = 0.005    ##< BEV 세로 1px 당 전방 거리[m]
    m_per_px_lateral: float = 0.005    ##< BEV 가로 1px 당 횡 거리[m]
    x_near_m: float = 0.15             ##< base_link(뒷차축)→BEV 최하단행 전방거리[m]

    # --- 색 신뢰도 정규화(이 픽셀수면 confidence=1.0) ---
    color_conf_pixels: float = 2000.0


@dataclass
class LaneResult:
    """@brief racer_msgs/LaneStatus 미러 + lane_path(미터). @see docs/interfaces.md §4.3."""

    lane_detected: bool = False
    confidence: float = 0.0
    num_points: int = 0
    lateral_offset: float = 0.0    ##< [m], +좌
    heading_error: float = 0.0     ##< [rad]
    stop_line: bool = False
    stop_line_dist: float = -1.0   ##< [m], 미검출 -1.0
    # 진단(계약 밖, 튜닝용): 왜 검출/미검출인지 숫자로 확인.
    stopline_cov_max: float = 0.0  ##< 정지선 ROI 행 커버리지 최댓값(0~1). row_coverage 임계와 비교.
    stopline_n_band: int = 0       ##< 커버리지 임계 넘은 행수. min_rows 임계와 비교.
    white_detected: bool = False
    white_confidence: float = 0.0
    # 팻말 강제 anchor 결과(계약 밖, 튜닝용). ''=지시 없음, 'applied:<side>',
    # 'gated:<side>:curve(...)'=커브 게이트가 보류, 'dropped:<side>:...'=앵커가 없어 포기.
    # 무증상 폴백을 눈에 보이게 하는 용도 — 노드가 그대로 로그로 찍는다.
    sign_force_status: str = ''
    # lane_path: (x,y) 미터, base_link, near→far
    lane_path: List[Tuple[float, float]] = field(default_factory=list)
    debug_image: Optional[np.ndarray] = None
    # 중간 파이프라인 단계(이름→이미지). want_debug일 때만 채움. 예: {'bev':..., 'edges':...}
    debug_stages: dict = field(default_factory=dict)


class LaneDetector:
    """@brief 프레임(BGR) → LaneResult. 상태(직전 좌/우 x, BEV 행렬)를 내부에 유지."""

    def __init__(self, calib: Optional[LaneCalib] = None):
        self.calib = calib or LaneCalib()
        self._M = None
        self._M_size: Optional[Tuple[int, int]] = None
        self._last_leftx = 0.0
        self._last_rightx = 0.0
        self._last_conf = 0.0        ##< 직전 프레임 confidence. seed lock/lost 판정용.
        self._lane_width_px = None   ##< 두 선 다 보일 때 학습한 차선폭[px]. None이면 config 기본값.
        # 단일 기준선 추종(single_anchor) 상태
        self._anchor_fit = None      ##< 직전 기준선 창별 x 배열(소실 hold coast용).
        self._anchor_age = 999       ##< 기준선 마지막 검출 이후 프레임(0=이번 프레임 검출).
        # 적응형 anchor(3-state) 상태
        self._lane_mode = 'dual'     ##< 현재 모드: 'dual' | 'left' | 'right'.
        self._pending_mode = None    ##< 전환 대기 중인 목표 모드(히스테리시스).
        self._pending_count = 0      ##< 목표 모드 연속 프레임 수.
        self._adapt_fit = None       ##< 적응형 단일 모드 anchor 선 coast용.
        self._adapt_age = 999        ##< 적응형 anchor 마지막 검출 이후 프레임.
        self._adapt_side = None      ##< _adapt_fit이 어느 쪽 선인지('left'|'right'). coast 오염 방지.
        self._straight_frames = 0    ##< 연속 '직선' 프레임 수(팻말 강제 허용 판정용).
        self._curve_dx_hist: List[float] = []  ##< curve_dx 최근값(중앙값 필터용, 검출된 것만).
        self.sign_force_status = ''  ##< 직전 프레임 팻말 강제 결과(LaneResult로 전달·로그).

    # ------------------------------------------------------------------ #
    def detect(self, frame: np.ndarray, want_debug: bool = False,
               force_side: Optional[str] = None,
               sign_offset_px: float = 0.0) -> LaneResult:
        """@brief 한 프레임 처리. @param frame BGR 이미지.
        @param force_side 판단(팻말)이 지시한 강제 anchor 쪽('left'|'right'|None).
                          None이면 adaptive_anchor 자동(커브/직선). 지정 시 커브·dual을
                          무시하고 그 쪽 차선만 추종한다.
        @param sign_offset_px 강제 anchor 시 차선중심에서 팻말 방향으로 더 붙일 오프셋[BEV px].
        @return LaneResult."""
        res = LaneResult()
        if frame is None or getattr(frame, 'size', 0) == 0:
            return res

        c = self.calib
        h, w = frame.shape[:2]
        midpoint = w // 2

        bev = self._to_bev(frame)
        hls = cv2.cvtColor(bev, cv2.COLOR_BGR2HLS)

        # 흰선-only 폐루프(07-14): 지름길(노랑 차선추종) 폐기 → 차선 마스크는 흰선만.
        # 노랑 마스크/래치/anchor-solidity는 전부 제거. (정지선 검출은 _stopline이
        # 자체 노랑 마스크를 따로 만들어 쓰므로 여기 흰선-only와 무관하게 유지된다.)
        white = self._white_mask(hls)
        white_px = int(cv2.countNonZero(white))

        res.white_detected = white_px > c.white_pixel_threshold
        res.white_confidence = min(1.0, white_px / c.color_conf_pixels)
        lane_mask = white

        edges = self._edges_from_mask(bev, lane_mask)

        debug = bev.copy() if want_debug else None

        # 디버그 오버레이(방식 A): 흰선 마스크를 슬라이딩윈도우와 '같은 화면'에서 대조할
        # 수 있게 debug(=BEV)에 반투명 초록으로 덧칠한다. 마스크는 이미 BEV 좌표라 정확히
        # 정렬된다. 여기서 먼저 칠하고 아래에서 박스/중심선을 그려 그 위에 올라오므로
        # 박스·선은 가려지지 않는다(디버그 전용, 제어에는 무관).
        if debug is not None:
            overlay = debug.copy()
            overlay[white > 0] = (0, 255, 0)      # 흰 마스크 → 초록
            cv2.addWeighted(overlay, 0.4, debug, 0.6, 0.0, dst=debug)

        # --- 중심선 점열(BEV px, near→far) + confidence ---
        # sign_apply="offset"이면 슬라이딩윈도우엔 팻말을 넘기지 않는다(기하학 순수 유지)
        # → 아래에서 최종 중심선만 평행이동. "anchor"면 옛 방식대로 anchor를 교체한다.
        _offset_mode = (c.sign_apply == 'offset')
        centerline_px, confidence = self._sliding_window(
            edges, debug,
            force_side=(None if _offset_mode else force_side),
            sign_offset_px=sign_offset_px)
        res.sign_force_status = self.sign_force_status   # 강제 발동/보류/포기 사유(로그용).

        # 팻말 offset: 기하학이 만든 중심선을 팻말 반대쪽(=지시된 통로 쪽)으로 평행이동.
        # 팻말이 도로 정중앙에 선 장애물이라, 중심선 그대로면 정면충돌 → 옆 통로로 민다.
        # 커브/직선 무관하게 안전(추종 로직 자체는 손대지 않으므로).
        if _offset_mode and force_side in ('left', 'right'):
            if centerline_px and sign_offset_px:
                _dx = float(sign_offset_px) * (1.0 if force_side == 'right' else -1.0)
                centerline_px = [(cx + _dx, cy) for (cx, cy) in centerline_px]
                # anchor 방식과 달리 여기엔 게이트도 '지시된 쪽 선 소실'도 없다 —
                # 차선이 잡히기만 하면 항상 발동한다. 그래도 로그는 남긴다(관측성 유지).
                res.sign_force_status = f'applied:{force_side}(offset {_dx:+.0f}px)'
            else:
                # 중심선 자체가 없음(차선 미검출) = 팻말 이전에 주행이 이미 실패한 상태.
                res.sign_force_status = f'dropped:{force_side}:중심선없음(차선 미검출)'
        res.confidence = confidence
        res.lane_detected = len(centerline_px) >= 2 and confidence > 0.0

        if res.lane_detected:
            path_m = [self._px_to_m(cx, cy, h, midpoint) for (cx, cy) in centerline_px]
            path_m = self._smooth_path(path_m)            # 노이즈 완화(다항식 피팅)
            res.lane_path = path_m
            res.num_points = len(path_m)
            xn, yn = path_m[0]
            xf, yf = path_m[-1]
            res.lateral_offset = yn                       # 최근접점 횡오차[m], +좌
            res.heading_error = math.atan2(yf - yn, xf - xn)  # 접선 방향 vs 전방
        else:
            # 미검출: 경로 비움(controller watchdog가 정지). 계약: num_points=0.
            res.lane_path = []
            res.num_points = 0

        # --- 정지선: 검출 + 거리[m] ---
        detected, row_px, cov_max, n_band = self._stopline(bev, debug)
        res.stop_line = detected
        res.stopline_cov_max = cov_max
        res.stopline_n_band = n_band
        if detected and row_px >= 0.0:
            res.stop_line_dist = c.x_near_m + (h - row_px) * c.m_per_px_forward
        else:
            res.stop_line_dist = -1.0

        if debug is not None:
            cv2.line(debug, (midpoint, 0), (midpoint, h), (255, 255, 0), 1)  # 중앙 기준선
            # lane_path(발행되는 중심선) 시각화: 미터 경로를 BEV 픽셀로 역변환해 초록
            # 폴리라인+점으로 그린다. near(바닥)은 주황 원으로 강조. 모니터 "Lane" 판에
            # 그대로 나온다(디버그 전용, 제어 동작에는 영향 없음).
            if res.lane_path:
                pts = [self._m_to_px(x, y, h, midpoint) for (x, y) in res.lane_path]
                for a, b in zip(pts[:-1], pts[1:]):
                    cv2.line(debug, a, b, (0, 255, 0), 2)
                for p in pts:
                    cv2.circle(debug, p, 3, (0, 255, 0), -1)
                cv2.circle(debug, pts[0], 5, (0, 128, 255), -1)  # near(최근접점) 강조
        res.debug_image = debug

        # 중간 단계 노출(모니터 디버그 화면용). BEV=원근변환, edges=Canny 결과.
        if want_debug:
            # LANE EDGE 판 좌상단에 마스크 모드 표기(흰선-only='wh'). 글자는 반드시
            # '복사본'에만 그린다 — 원본 edges는 위 _sliding_window 탐지 입력이라 글자
            # 획이 가짜 에지로 섞이면 탐지가 오염된다(디버그 전용, 제어 무관).
            edges_dbg = edges.copy()
            mode_label = 'wh'
            cv2.putText(edges_dbg, mode_label, (6, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 2, cv2.LINE_AA)
            res.debug_stages = {'bev': bev, 'edges': edges_dbg}

        return res

    # ------------------------------------------------------------------ #
    def _white_mask(self, hls: np.ndarray) -> np.ndarray:
        """@brief 흰색 마스크. 적응형이면 프레임 밝기에 상대적인 L 하한을 쓴다.

        @param hls BEV 이미지의 HLS 변환(H,L,S).
        @return uint8 마스크(흰=255).

        @details 흰 차선은 항상 프레임 내 최고 밝기다. L 하한을 'L 상위 퍼센타일 ×
        비율'로 잡으면 조명이 밝든 어둡든 흰 선만 안정적으로 남아 깜빡임이 사라지고,
        상대적으로 어두운 회색 바닥·매트는 자동 배제된다(고정 임계의 밝음=회색오검 /
        어두움=흰선실종 문제를 동시 해결). S 상한으로 노랑 등 유채색을 배제한다.
        white_adaptive=False면 기존 고정 임계(white_lo/hi)로 폴백한다.
        """
        c = self.calib
        if not c.white_adaptive:
            return cv2.inRange(hls, np.array(c.white_lo), np.array(c.white_hi))
        L = hls[:, :, 1]
        S = hls[:, :, 2]
        l_lo = int(round(c.white_adaptive_k * float(np.percentile(L, c.white_pct))))
        l_lo = int(np.clip(l_lo, c.white_l_floor, c.white_l_cap))
        return ((L >= l_lo) & (S <= c.white_s_hi)).astype(np.uint8) * 255

    # ------------------------------------------------------------------ #
    def _edges_from_mask(self, bev: np.ndarray, lane_mask: np.ndarray) -> np.ndarray:
        """@brief 차선 마스크 → Canny 에지.

        @param bev       BEV 이미지(edge_from_mask=False 폴백 경로에서만 사용).
        @param lane_mask 노랑/흰 차선 이진 마스크(uint8, 차선=255).
        @return uint8 에지 이미지.

        @details 기본(edge_from_mask): 마스크에 MORPH_CLOSE로 작은 구멍을 메우고
        dilate로 살짝 두껍게 한 뒤 Canny → 차선 윤곽이 연속·깨끗해져 슬라이딩 윈도우
        창별 minpix가 안정된다(원인: 얇은 마스크+높은 Canny 임계면 윤곽이 끊겨
        '인식했다 말았다'). edge_from_mask=False면 옛 경로(BGR 마스킹→gray→blur→Canny).
        """
        c = self.calib
        if c.edge_from_mask:
            k = np.ones((max(1, c.edge_close_ksize),) * 2, np.uint8)
            m = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, k, iterations=1)
            if c.edge_dilate_iter > 0:
                m = cv2.dilate(m, k, iterations=c.edge_dilate_iter)
            return cv2.Canny(m, c.canny_lo, c.canny_hi)
        masked = cv2.bitwise_and(bev, bev, mask=lane_mask)
        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        return cv2.Canny(blur, c.canny_lo, c.canny_hi)

    # ------------------------------------------------------------------ #
    def _to_bev(self, frame: np.ndarray) -> np.ndarray:
        """@brief 원근변환(BEV). 이미지 크기 바뀌면 행렬 재계산."""
        h, w = frame.shape[:2]
        if self._M is None or self._M_size != (w, h):
            c = self.calib
            if c.bev_matrix is not None and len(c.bev_matrix) == 9:
                # 체커보드 IPM 실측 행렬 직접 사용(이미지→BEV, 행우선 3x3).
                self._M = np.array(c.bev_matrix, dtype=np.float32).reshape(3, 3)
            else:
                src = np.float32([
                    [w * c.bev_top_x, h * c.bev_top_y],
                    [w * (1.0 - c.bev_top_x), h * c.bev_top_y],
                    [w, h],
                    [0, h],
                ])
                dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
                self._M = cv2.getPerspectiveTransform(src, dst)
            self._M_size = (w, h)
            self._last_leftx = w * 0.25
            self._last_rightx = w * 0.75
        return cv2.warpPerspective(frame, self._M, (w, h))

    def _reacquire_base(self, hist: np.ndarray, w: int, h: int) -> Tuple[int, int]:
        """@brief lost 상태에서 좌/우 탐색 시작 base x를 재획득한다.

        @param hist 하단 영역 열별 에지 합(길이 w).
        @param w,h  프레임 크기[px].
        @return (leftx, rightx) 시작 base.

        @details 각 반쪽 히스토그램 peak의 에지량이 충분하면(=차선이 보이면)
        argmax를 직전값과 블렌딩해 채택한다. 임계 미만이면 그쪽 차선이 '안 보이는'
        것으로 보고 base를 화면 25%/75% 기본위치로 고정한다 — argmax는 에지가 없어도
        무조건 인덱스를 반환(전부 0이면 0=맨왼쪽)하므로 엉뚱한 점을 잡는 것을 막는다.
        """
        c = self.calib
        midpoint = w // 2
        n_rows = h - int(h * 0.6)                        # 히스토그램 스캔 행 수
        min_mass = 255.0 * n_rows * c.seed_min_fill      # 실제 차선 인정 최소 에지량
        left_hist = hist[:midpoint]
        right_hist = hist[midpoint:]
        b = c.seed_reacquire_blend                        # 히스토그램 가중치(prev와 블렌딩)
        if left_hist.size and float(left_hist.max()) >= min_mass:
            cur_left = int(np.argmax(left_hist))
            leftx = int(self._last_leftx * (1.0 - b) + cur_left * b)
        else:
            leftx = int(w * 0.25)                         # 좌측 차선 미검출 → 기본 25%
        if right_hist.size and float(right_hist.max()) >= min_mass:
            cur_right = int(np.argmax(right_hist) + midpoint)
            rightx = int(self._last_rightx * (1.0 - b) + cur_right * b)
        else:
            rightx = int(w * 0.75)                        # 우측 차선 미검출 → 기본 75%
        return leftx, rightx

    def _blend_lock_seed(self, prev_x: float, hist: np.ndarray, w: int, h: int) -> int:
        """@brief lock 상태 seed를 히스토그램 peak 쪽으로 소폭 당긴다(gated+windowed).

        @param prev_x 직전 seed x(=lock 시작점).
        @param hist   하단 영역 열별 에지 합(길이 w).
        @param w,h    프레임 크기[px].
        @return 블렌딩된 seed x.

        @details prev_x ±seed_hist_band_px 밴드 **안에서만** peak를 찾는다(윈도우
        탐색 → 반대 차선/노이즈로 seed가 점프하는 것을 원천 배제). 밴드 내 최대
        에지량이 seed_min_fill 기준 미만이면 그 프레임엔 신뢰할 peak가 없다고 보고
        prev_x를 그대로 유지한다(garbage 프레임 미반영). 기준을 넘으면
        prev*(1-b) + peak*b 로 섞어 커브에서 seed가 뒤처지는 지연만 줄인다.
        seed_lock_hist_blend=0이면 호출되지 않는다.
        """
        c = self.calib
        n_rows = h - int(h * 0.6)
        min_mass = 255.0 * n_rows * c.seed_min_fill
        lo = max(0, int(prev_x - c.seed_hist_band_px))
        hi = min(w, int(prev_x + c.seed_hist_band_px))
        if hi - lo < 1:
            return int(prev_x)
        band = hist[lo:hi]
        if float(band.max()) < min_mass:
            return int(prev_x)                        # 밴드 내 실제 에지 없음 → 유지
        peak = lo + int(np.argmax(band))
        b = c.seed_lock_hist_blend
        return int(prev_x * (1.0 - b) + peak * b)

    @staticmethod
    def _curve_dx(left_line, right_line, left_ok, right_ok):
        """@brief 커브 방향/세기 = 중심선(또는 검출된 한 선)의 near→far x 변화량[px].

        @return +우커브(far가 오른쪽) / −좌커브 / None(판단 불가). |값|이 클수록 급커브.
        """
        if left_ok and right_ok:
            ref = (left_line + right_line) / 2.0
        elif left_ok:
            ref = left_line
        elif right_ok:
            ref = right_line
        else:
            return None
        return float(ref[-1] - ref[0])

    def _anchor_centerline(self, use_left, anchor_line, anchor_ok, lane_w,
                           cys, cys_arr, lfound, rfound, midpoint, leftx, rightx,
                           extra_offset_px=0.0):
        """@brief 단일 기준선(바깥/강제) → 중심선. 소실 시 coast, perp 오프셋, seed 갱신.

        @param use_left True=왼선 anchor(중심은 오른쪽 +W/2), False=오른선(−W/2).
        @param extra_offset_px 차선중심에서 추가로 실을 오프셋[px](팻말 방향 붙임). 부호 포함.
        @return (centerline_px, confidence). adaptive 단일모드와 강제(팻말) 모드가 공유.
        """
        c = self.calib
        side = 'left' if use_left else 'right'
        anchor_side = side
        if anchor_ok:
            self._adapt_fit = anchor_line              # 실검출 → 갱신
            self._adapt_age = 0
            self._adapt_side = side
            n_side = sum(lfound if use_left else rfound)
            conf = n_side / float(c.nwindows)
            anchor = anchor_line
        elif (self._adapt_fit is not None
              and self._adapt_age < c.anchor_hold_frames):
            # 소실: 직전 anchor 유지(coast). 측이 방금 바뀌었어도(W자 변곡점에서
            # curve_dx 부호 반전) 반대쪽 fit을 **그 fit의 측 기준으로** 복원해 쓴다.
            # 좌/우 anchor 모두 같은 차선중심을 가리키므로 기하는 그대로 유효하고,
            # 새 쪽이 잡힐 때까지 중심선이 midpoint로 스냅되는 계단이 생기지 않는다.
            anchor = self._adapt_fit
            anchor_side = self._adapt_side
            self._adapt_age += 1
            conf = max(c.seed_lock_conf, self._last_conf * 0.9)
        else:
            anchor = None                              # 완전 소실 → 직진 폴백
            conf = 0.0
        if anchor is not None:
            half = lane_w / 2.0
            anchor_is_left = (anchor_side == 'left')   # coast 시 요청 측과 다를 수 있음
            sign = 1.0 if anchor_is_left else -1.0
            if c.perp_offset:
                slope = np.gradient(anchor, cys_arr)   # dx/dy(row별)
                slope = np.clip(slope, -c.perp_max_slope, c.perp_max_slope)
                scale = np.sqrt(1.0 + slope * slope)
            else:
                scale = 1.0
            cxs = anchor + sign * half * scale + extra_offset_px
            centerline = [(float(cxs[i]), cys[i]) for i in range(c.nwindows)]
            near = float(anchor[0])
            if anchor_is_left:
                self._last_leftx, self._last_rightx = near, near + lane_w
            else:
                self._last_rightx, self._last_leftx = near, near - lane_w
        else:
            centerline = [(float(midpoint), cys[i]) for i in range(c.nwindows)]
            self._last_leftx, self._last_rightx = leftx, rightx
        if c.lane_width_learn and lane_w > 1.0:
            self._lane_width_px = lane_w
        self._last_conf = conf
        return centerline, conf

    def _sliding_window(self, edges: np.ndarray, debug,
                        force_side=None, sign_offset_px=0.0):
        """@brief 좌/우 차선 슬라이딩 윈도우. @return (centerline_px[near→far], confidence).

        @param force_side 판단(팻말) 강제 anchor 쪽('left'|'right'|None). 지정 시 커브감지·
                          dual·single_anchor를 모두 무시하고 그 쪽 차선만 추종.
        @param sign_offset_px 강제 anchor 시 팻말 방향으로 추가로 붙일 오프셋[px](>=0 크기).
        """
        c = self.calib
        h, w = edges.shape[:2]
        midpoint = w // 2

        # 시작 x seed 결정: lock(추종 중)이면 직전 피팅 near-x를 그대로 써서 곡선을
        # 연속 추종(search-around-poly). lost면 하단 히스토그램으로 좌/우 재획득.
        # midpoint 고정분할은 곡선에서 좌우를 뒤바꾸므로 lost일 때만 경로에 탄다.
        if self._last_conf >= c.seed_lock_conf:
            leftx = int(self._last_leftx)
            rightx = int(self._last_rightx)
            if c.seed_lock_hist_blend > 0.0:
                # 옵션: lock 중에도 히스토그램 peak를 소폭 섞어 커브 지연을 줄인다.
                hist = np.sum(edges[int(h * 0.6):, :], axis=0)
                leftx = self._blend_lock_seed(self._last_leftx, hist, w, h)
                rightx = self._blend_lock_seed(self._last_rightx, hist, w, h)
        else:
            hist = np.sum(edges[int(h * 0.6):, :], axis=0)
            leftx, rightx = self._reacquire_base(hist, w, h)

        nz = cv2.findNonZero(edges)
        centerline: List[Tuple[float, float]] = []
        if nz is None or len(nz) == 0:
            self._last_conf = 0.0   # 에지 없음 → lost. 다음 프레임 히스토그램 재획득.
            return centerline, 0.0
        # findNonZero는 OpenCV 버전에 따라 (N,1,2) 또는 (N,2) → 항상 (N,2)로.
        pts = np.asarray(nz).reshape(-1, 2)
        nzx = pts[:, 0]
        nzy = pts[:, 1]

        window_height = max(1, h // c.nwindows)

        # --- 1단계: 좌/우 각각 추적(창별 위치 + 검출여부 기록) ---
        lxs: List[float] = []
        rxs: List[float] = []
        cys: List[float] = []
        lfound: List[bool] = []
        rfound: List[bool] = []
        for win in range(c.nwindows):
            y_low = h - (win + 1) * window_height
            y_high = h - win * window_height
            lx_low, lx_high = leftx - c.margin, leftx + c.margin
            rx_low, rx_high = rightx - c.margin, rightx + c.margin

            in_y = (nzy >= y_low) & (nzy < y_high)
            good_left = in_y & (nzx >= lx_low) & (nzx < lx_high)
            good_right = in_y & (nzx >= rx_low) & (nzx < rx_high)

            lf = int(good_left.sum()) > c.minpix
            rf = int(good_right.sum()) > c.minpix
            if lf:
                leftx = int(nzx[good_left].mean())
            if rf:
                rightx = int(nzx[good_right].mean())

            lxs.append(leftx); rxs.append(rightx); cys.append((y_low + y_high) / 2.0)
            lfound.append(lf); rfound.append(rf)

            if debug is not None:
                # 엣지를 실제로 잡은 창만 그린다. lf/rf 무관하게 그리면 엣지가 없어도
                # seed 위치(lock=직전 seed, lost=0.25/0.75w 기본)에 유령 윈도우가 뜬다.
                # single_anchor면 '실제 쓰는' 기준선 창만 표시(대시보드 명확화).
                draw_left = (not c.single_anchor) or (c.anchor_side == 'left')
                draw_right = (not c.single_anchor) or (c.anchor_side != 'left')
                if lf and draw_left:
                    cv2.rectangle(debug, (lx_low, y_low), (lx_high, y_high), (255, 0, 0), 2)
                if rf and draw_right:
                    cv2.rectangle(debug, (rx_low, y_low), (rx_high, y_high), (0, 0, 255), 2)

        # --- 2단계: 유효 선 판정 + 곡선 피팅(far까지 연장) ---
        # 잡힌 점들을 x=f(y) 다항식으로 피팅해 놓친(위쪽) 창까지 곡선을 연장한다
        # → 커브에서 선이 프레임 위로 빠져도 중심선이 계속 휘어 커브를 완주.
        # min_track = 피팅에 필요한 최소 창 수. 실제 트랙엔 얼룩이 없어 느슨히 2로 둠
        # (2 미만은 피팅 불가). 얼룩 있는 환경이면 ↑로 얼룩 오검을 컷할 수 있음.
        min_track = 2
        cys_arr = np.array(cys, dtype=np.float64)

        def fit_line(found_flag, xs_list):
            ys = np.array([cys[i] for i in range(c.nwindows) if found_flag[i]],
                          dtype=np.float64)
            xs = np.array([xs_list[i] for i in range(c.nwindows) if found_flag[i]],
                          dtype=np.float64)
            if len(ys) < min_track or float(ys.max() - ys.min()) < 1e-3:
                return None
            order = 2 if len(ys) >= 3 else 1   # 3점 이상이면 곡선(2차)
            try:
                coef = np.polyfit(ys, xs, order)
            except (np.linalg.LinAlgError, ValueError):
                return None
            return np.polyval(coef, cys_arr)   # 모든 창 y에서의 x(연장 포함)

        left_line = fit_line(lfound, lxs)
        right_line = fit_line(rfound, rxs)
        left_ok = left_line is not None
        right_ok = right_line is not None

        # 차선폭 학습(양쪽 다 피팅될 때, 피팅선 간격 평균).
        if c.lane_width_learn:
            lane_w = self._lane_width_px if self._lane_width_px is not None \
                else float(c.lane_width_px)
            if left_ok and right_ok:
                wobs = float(np.mean(right_line - left_line))
                if wobs > 0.3 * lane_w:
                    lane_w = lane_w * 0.7 + wobs * 0.3
        else:
            lane_w = float(c.lane_width_px)   # 학습 off: 항상 고정값(표류 없음)

        # --- 강제 anchor(팻말 지시): 커브감지·dual·single_anchor 전부 무시하고 왼선 기준 ---
        # 판단이 방향 팻말(좌/우)을 래치해 force_side로 내려주면, **왼선 하나를 기준선으로**
        # 중심선 = 왼선 + W/2 ± sign_offset_px 를 만든다(−=왼쪽 통로, +=오른쪽 통로).
        # 팻말이 풀리면(force_side=None) 아래 adaptive/single 자동 로직으로 복귀한다.
        # --- 커브/직선 판정 + 연속 직선 카운터(팻말 강제 게이트용) ---
        # 매 프레임 갱신한다(팻말 유무와 무관). 커브면 카운터를 0으로 리셋하므로,
        # S자 변곡점처럼 '찰나만 직선'인 구간은 카운터가 못 쌓여 팻말이 차단된다.
        self.sign_force_status = ''                    # 이번 프레임 팻말 강제 결과(노드가 로그).
        curve_dx_raw = self._curve_dx(left_line, right_line, left_ok, right_ok)
        # 중앙값 필터: 원거리 끝점 노이즈(부호까지 뒤집힘)를 걸러 '진짜 커브'만 남긴다.
        # 검출된 값만 쌓는다 — 미검출(None)을 0으로 채우면 '직선'을 지어내는 셈이라
        # 차선이 안 보이는 커브가 직선으로 둔갑한다. None은 옛 동작대로 '판단 불가'.
        if curve_dx_raw is None:
            curve_dx = None
        else:
            n = max(1, int(c.sign_curve_median_frames))
            self._curve_dx_hist.append(float(curve_dx_raw))
            if len(self._curve_dx_hist) > n:
                del self._curve_dx_hist[:-n]           # 최근 n개만 유지
            curve_dx = float(np.median(self._curve_dx_hist))
        if curve_dx is not None and abs(curve_dx) > c.sign_force_max_curve_px:
            self._straight_frames = 0                  # 커브 → 리셋
        else:
            self._straight_frames += 1                 # 직선(또는 판단불가) → 누적

        if force_side in ('left', 'right'):
            # 커브 게이트: '직선이 sign_force_straight_frames 연속' 이어야 팻말을 허용.
            # 커브 중이거나, 변곡점처럼 직선이 잠깐뿐이면 → 기하학(adaptive) 추종 유지.
            if (c.sign_force_max_curve_px > 0.0
                    and self._straight_frames < c.sign_force_straight_frames):
                _dx = 'None' if curve_dx is None else f'{curve_dx:.0f}'
                _raw = 'None' if curve_dx_raw is None else f'{curve_dx_raw:.0f}'
                self.sign_force_status = (          # dx=중앙값(게이트가 보는 값), raw=원본.
                    f'gated:{force_side}:curve(dx={_dx}/{c.sign_force_max_curve_px:.0f} '
                    f'raw={_raw} straight={self._straight_frames}/'
                    f'{c.sign_force_straight_frames})')
                force_side = None                      # 아직 직선 확정 아님 → 자동 로직으로.

        if force_side in ('left', 'right'):
            # [07-16] 좌/우 **둘 다 왼선 앵커**. 예전엔 지시된 쪽 선을 그대로 앵커로 썼는데,
            # 오른쪽 분기가 필요한 바로 그 지점(S자 끝 = 우커브)에서 영영 발동하지 못했다:
            # 우커브에선 '안쪽'인 오른선이 BEV 밖으로 먼저 빠지고(아래 adaptive 기하 주석
            # 참조) right_ok=False, coast는 _adapt_side가 'left'(adaptive가 우커브에서
            # 바깥=왼선을 잡으니까)라 조건 불일치로 안 걸린다 → 강제 포기 → adaptive
            # (왼선, offset 0) = 도로 정중앙 = 팻말 정면. **좌/우 팻말이 똑같이 왼쪽으로
            # 가던 원인이 이것**이다. 보이는 선 하나로 양방향을 표현하면 이 비대칭
            # (coast 래치 편향 + 안쪽선 소실)이 통째로 사라진다.
            anchor_line, anchor_ok = left_line, left_ok
            can_coast = (self._adapt_fit is not None
                         and self._adapt_side == 'left'
                         and self._adapt_age < c.anchor_hold_frames)
            if anchor_ok or can_coast:
                self._lane_mode = 'left'               # 모니터/일관성용 모드 반영
                # 왼선 기준이므로 부호는 '팻말의 어느 쪽 통로냐'로 갈린다(오른선 앵커 시절의
                # use_left 부호가 아니다): 왼쪽 통로=−(왼선 쪽), 오른쪽 통로=+(반대편).
                extra = (-1.0 if force_side == 'left' else 1.0) * float(sign_offset_px)
                self.sign_force_status = f'applied:{force_side}'
                return self._anchor_centerline(
                    True, anchor_line, anchor_ok, lane_w, cys, cys_arr,
                    lfound, rfound, midpoint, leftx, rightx, extra_offset_px=extra)
            # 왼선마저 없고 coast도 만료 → 강제 포기(없는 선을 좇으면 conf=0 → 판단
            # LOST → 정지). 단 **조용히 버리지 않는다**: 이 무증상 폴백이 좌/우 동일
            # 증상을 모델 탓으로 오진하게 만든 장본인이라 사유를 남겨 노드가 찍는다.
            self.sign_force_status = f'dropped:{force_side}:왼선없음(coast만료)'

        # --- 단일 기준선 고정 추종(single_anchor): 오른선 하나만, 붕괴/정체성 우회 ---
        if c.single_anchor:
            # anchor 경계 = 설정값 c.anchor_side(메인: 오른선 화면밖→left).
            anchor_side = c.anchor_side
            right = (anchor_side != 'left')            # 오른쪽 경계선 여부
            anchor = right_line if right else left_line
            anchor_ok = right_ok if right else left_ok
            if anchor_ok:
                self._anchor_fit = anchor              # 이번 프레임 실검출 → 갱신
                self._anchor_age = 0
                n_side = sum(rfound if right else lfound)
                conf = n_side / float(c.nwindows)
            elif self._anchor_fit is not None and self._anchor_age < c.anchor_hold_frames:
                anchor = self._anchor_fit              # 소실: 직전 기준선 유지(점선 갭 coast)
                self._anchor_age += 1
                conf = max(c.seed_lock_conf, self._last_conf * 0.9)  # hold 중 lock 유지
            else:
                anchor = None                          # 완전 소실: 직진 폴백
                conf = 0.0
            if anchor is not None:
                bias = (-lane_w / 2.0) if right else (lane_w / 2.0)  # 오른선→중심은 왼쪽(-)
                cxs = anchor + bias
                centerline = [(float(cxs[i]), cys[i]) for i in range(c.nwindows)]
                near = float(anchor[0])
                if right:
                    self._last_rightx, self._last_leftx = near, near - lane_w
                else:
                    self._last_leftx, self._last_rightx = near, near + lane_w
            else:
                centerline = [(float(midpoint), cys[i]) for i in range(c.nwindows)]
                self._last_leftx, self._last_rightx = leftx, rightx
            if c.lane_width_learn and lane_w > 1.0:
                self._lane_width_px = lane_w
            self._last_conf = conf
            return centerline, conf

        # --- 적응형 anchor(3-state): 직선/양선=dual, 커브=바깥선 single 자동 전환 ---
        # 기하: 커브에서 '안쪽' 선이 BEV 옆으로 먼저 빠지고 '바깥'(커브 반대쪽) 선이
        # 프레임에 남는다 → 오른쪽 커브=왼선 anchor, 왼쪽 커브=오른선 anchor. 직선·양선
        # 뚜렷하면 dual(양선 중앙). 커브에선 안쪽(노이즈·소실) 선을 무시해 dual 붕괴/
        # 조기치우침을 원천 차단. 모드 전환은 히스테리시스로 채터링 방지.
        if c.adaptive_anchor:
            # 1) 커브 방향/세기: 위에서 계산한 curve_dx 재사용.
            #    curve_dx>0=우커브(far가 오른쪽), <0=좌커브. |·|가 deadband 이하면 직선.

            both_clean = (left_ok and right_ok and
                          float(np.mean(right_line - left_line))
                          >= c.lane_collapse_frac * lane_w)

            # 2) 목표 모드.
            if curve_dx is None:
                desired = None                              # 판단 불가 → 현재 유지
            elif abs(curve_dx) <= c.curve_dx_deadband_px:   # 직선
                if both_clean:
                    desired = 'dual'
                elif left_ok and not right_ok:
                    desired = 'left'
                elif right_ok and not left_ok:
                    desired = 'right'
                else:
                    desired = None
            elif curve_dx > 0.0:
                desired = 'left'                            # 우커브 → 바깥(왼) 선
            else:
                desired = 'right'                           # 좌커브 → 바깥(오른) 선

            # 3) 히스테리시스 커밋: dual 승격은 both_enter_frames, 그 외 전환은
            #    both_exit_frames 연속 프레임을 요구(1~2프레임 점선 갭·노이즈에 안 흔들림).
            if desired is None or desired == self._lane_mode:
                self._pending_mode = None
                self._pending_count = 0
            else:
                if desired == self._pending_mode:
                    self._pending_count += 1
                else:
                    self._pending_mode = desired
                    self._pending_count = 1
                need = c.both_enter_frames if desired == 'dual' else c.both_exit_frames
                if self._pending_count >= need:
                    self._lane_mode = desired
                    self._pending_mode = None
                    self._pending_count = 0

            # 4) 단일(바깥선) 모드면 헬퍼로 중심선 산출·반환(강제모드와 공유). dual이면 낙하.
            if self._lane_mode in ('left', 'right'):
                use_left = (self._lane_mode == 'left')
                anchor_line = left_line if use_left else right_line
                anchor_ok = left_ok if use_left else right_ok
                return self._anchor_centerline(
                    use_left, anchor_line, anchor_ok, lane_w, cys, cys_arr,
                    lfound, rfound, midpoint, leftx, rightx)
            # self._lane_mode == 'dual' → 아래 붕괴방지 + 3단계 양선중심 경로로 낙하.

        # --- 붕괴(collapse) 방지 ---
        # 노이즈/한쪽 선 끊김으로 좌·우 두 탐색창이 같은 실선 하나에 달라붙으면
        # (두 피팅선 간격 < lane_collapse_frac × lane_w) 중심선이 반차선 튀고, seed도
        # 같은 선에 얹혀 lock인 채 못 빠져나온다(→ 원래 차선 상실·이탈). 이때 단일
        # 차선으로 강등: 붕괴 위치가 직전 좌/우 seed 중 어디에 더 가깝냐로 어느 실선인지
        # 판정하고 반대편을 lane_w로 복원한다 → 중심선 정상화 + 다음 seed 분리(복귀).
        if left_ok and right_ok and \
                float(np.mean(right_line - left_line)) < c.lane_collapse_frac * lane_w:
            merged = (left_line + right_line) / 2.0            # 붙어버린 실선(창별)
            if abs(float(merged[0]) - self._last_leftx) <= \
                    abs(float(merged[0]) - self._last_rightx):
                left_line, right_line = merged, merged + lane_w    # 왼쪽 선에 붙음
            else:
                left_line, right_line = merged - lane_w, merged    # 오른쪽 선에 붙음

        # --- 3단계: 모드별 중심선(피팅선 사용 → 곡선 연장) ---
        centerline = []
        prev_cx = None
        for i in range(c.nwindows):
            if left_ok and right_ok:
                cx = (left_line[i] + right_line[i]) / 2.0
            elif left_ok:                       # 왼쪽선만: 오른쪽을 차선폭으로 복원
                cx = left_line[i] + lane_w / 2.0
            elif right_ok:                      # 오른쪽선만: 왼쪽 복원
                cx = right_line[i] - lane_w / 2.0
            else:
                cx = prev_cx if prev_cx is not None else float(midpoint)
            centerline.append((float(cx), cys[i]))
            prev_cx = cx

        # confidence: 실제로 픽셀이 잡힌 창 비율(피팅 연장분 제외).
        found_any = sum(1 for i in range(c.nwindows) if lfound[i] or rfound[i])
        confidence = found_any / float(c.nwindows)

        # 다음 프레임 seed: near(바닥=cys[0]) 쪽 피팅값 우선. 한쪽만 잡힌 경우
        # 놓친 쪽 seed를 (잡힌 선 ± lane_w)로 앵커한다 → 곡선에서 한쪽만 보다가
        # 반대편 차선이 돌아올 때 탐색창이 이미 예측 위치에 있어 즉시 재획득한다.
        # (앵커 없이 leftx/rightx를 그대로 두면 놓친 쪽 seed가 히스토그램 노이즈로
        #  표류해, 복귀 프레임에서 진짜 선을 margin 밖에 두고 몇 프레임간 못 잡음.)
        if left_ok and right_ok:
            self._last_leftx = float(left_line[0])
            self._last_rightx = float(right_line[0])
        elif left_ok:
            self._last_leftx = float(left_line[0])
            self._last_rightx = float(left_line[0] + lane_w)
        elif right_ok:
            self._last_rightx = float(right_line[0])
            self._last_leftx = float(right_line[0] - lane_w)
        else:
            self._last_leftx = leftx
            self._last_rightx = rightx
        if c.lane_width_learn and lane_w > 1.0:
            self._lane_width_px = lane_w   # 다음 프레임으로 학습 폭 이월(학습 off면 미이월).
        self._last_conf = confidence       # 다음 프레임 seed lock/lost 판정에 사용.
        return centerline, confidence

    def _smooth_path(self, path_m: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """@brief 중심선(x,y)을 다항식(y~x)으로 피팅해 창별 노이즈 제거.

        @details 슬라이딩 윈도우 중심은 창마다 흔들려 조향 휘청임을 유발한다.
        near→far로 단조증가하는 x에 대해 y를 저차 다항식으로 피팅해 매끄러운 경로를
        만든다. 점이 부족하거나 x폭이 없으면 원본 유지(안전).
        """
        c = self.calib
        if not c.path_smooth or len(path_m) < c.path_smooth_order + 1:
            return path_m
        xs = np.array([p[0] for p in path_m], dtype=np.float64)
        ys = np.array([p[1] for p in path_m], dtype=np.float64)
        if float(xs.max() - xs.min()) < 1e-3:             # x가 거의 동일 → 피팅 불가
            return path_m
        try:
            coef = np.polyfit(xs, ys, c.path_smooth_order)
            ys_fit = np.polyval(coef, xs)
        except (np.linalg.LinAlgError, ValueError):
            return path_m
        return [(float(x), float(y)) for x, y in zip(xs, ys_fit)]

    def _px_to_m(self, col: float, row: float, h: int, midpoint: int) -> Tuple[float, float]:
        """@brief BEV 픽셀(col,row) → base_link 미터(+x 전방, +y 좌). 잠정 스케일."""
        c = self.calib
        x = c.x_near_m + (h - row) * c.m_per_px_forward
        y = (midpoint - col) * c.m_per_px_lateral
        return (x, y)

    def _m_to_px(self, x: float, y: float, h: int, midpoint: int) -> Tuple[int, int]:
        """@brief base_link 미터(x,y) → BEV 픽셀(col,row). `_px_to_m`의 역변환(디버그 그리기용)."""
        c = self.calib
        col = midpoint - y / c.m_per_px_lateral if c.m_per_px_lateral else midpoint
        row = h - (x - c.x_near_m) / c.m_per_px_forward if c.m_per_px_forward else h
        return (int(round(col)), int(round(row)))

    @staticmethod
    def _longest_run(row_bool: np.ndarray) -> int:
        """@brief 불리언 1행에서 True가 '연속'으로 이어진 최대 길이[px]. @return int.
        @details 정지선 가로폭(연속 색 띠 길이) 측정용. 경계에 0을 덧대 diff로 run
        경계를 찾고 (끝-시작) 최댓값을 취한다. 빈 행이면 0.
        """
        if not row_bool.any():
            return 0
        edges = np.diff(np.concatenate(([0], row_bool.astype(np.int8), [0])))
        starts = np.nonzero(edges == 1)[0]
        ends = np.nonzero(edges == -1)[0]
        return int((ends - starts).max())

    def _stopline_hough(self, mask: np.ndarray, y0: int, h: int, w: int, debug):
        """@brief 정지선 Hough 검출(작년 검증 방식). @return (detected, nearest_row|-1, diag_len, n_seg).

        @param mask 정지선색 이진 마스크(하단 ROI 좌표). @param y0 ROI 상단의 BEV 행.
        @details 마스크에 Canny→HoughLinesP로 선분을 뽑고 '수평'(|angle|<
        stopline_angle_tol_deg 또는 >180-tol) 선분 길이를 합산, stopline_len_threshold를
        넘으면 검출. 세로 차선은 각도로 배제된다. 거리용 최근접 행은 수평 선분 중 가장
        바닥(y 큰)을 ROI→BEV로 보정(+y0). 진단: (총 수평길이/폭, 수평 선분 수).
        """
        c = self.calib
        edges = cv2.Canny(mask, int(c.stopline_canny_lo), int(c.stopline_canny_hi))
        lines = cv2.HoughLinesP(
            edges, 1, math.pi / 180.0, int(c.stopline_hough_thresh),
            minLineLength=int(c.stopline_hough_min_len),
            maxLineGap=int(c.stopline_hough_max_gap))
        tol = float(c.stopline_angle_tol_deg)
        total_len = 0.0
        seg_ys: List[float] = []
        if lines is not None:
            for ln in lines:
                # HoughLinesP 반환 형태가 OpenCV 버전따라 (N,1,4)/(N,4) → ravel로 통일.
                x1, y1, x2, y2 = (int(v) for v in np.asarray(ln).reshape(-1)[:4])
                ang = abs(math.degrees(math.atan2(y2 - y1, x2 - x1)))
                if ang < tol or ang > 180.0 - tol:      # 수평 선분만(세로 차선 배제)
                    total_len += math.hypot(x2 - x1, y2 - y1)
                    seg_ys.append((y1 + y2) / 2.0)
                    if debug is not None:
                        cv2.line(debug, (x1, y1 + y0), (x2, y2 + y0), (0, 255, 255), 2)
        detected = total_len > float(c.stopline_len_threshold)
        nearest_row = float(max(seg_ys) + y0) if (detected and seg_ys) else -1.0
        diag_len = total_len / float(max(1, w))          # 진단: 폭 대비 총 수평길이
        return (detected, nearest_row, diag_len, len(seg_ys))

    def _stopline(self, bev: np.ndarray, debug):
        """@brief 정지선 검출 + 최근접 정지선의 BEV 행(row). @return (detected, row|-1).

        @details 하단 ROI에서 **행별 정지선색 커버리지**로 검출한다. 색은
        stopline_color(yellow|white|both, 실트랙 기본 yellow). 정지선은 화면을
        가로지르는 색 띠라 각 행의 정지선색 픽셀이 폭의 큰 비율을 덮는다(row_coverage↑).
        반면 차선은 BEV에서 세로라(같은 노랑이라도) 한 행에서 좁게만 걸려 커버리지가
        낮다 → 차선/노이즈와 강하게 분리된다. 후보 행이 min_rows 이상이면 검출, 가장
        바닥에 가까운(=row 큰) 후보 행을 거리 산출용 최근접 행으로 반환한다.
        debug가 있으면 검출 밴드에 빨간 박스를 그려 인식 여부를 눈으로 확인할 수 있다.
        """
        c = self.calib
        h, w = bev.shape[:2]
        # stopline_roi_h <= 0 이면 하단 ROI 없이 BEV '전체'에서 정지선 탐색(전체화면 모드).
        # >0 이면 하단 그 높이[px]만 검사(기존 동작). y0 오프셋은 아래 좌표 복원(nearest_row,
        # hough)에 그대로 쓰이므로 두 경로 모두 정확하다. 전체화면은 원거리/커브에서 가로로
        # 눕는 차선·먼 정지선까지 잡을 수 있어 조기검출↑ 대신 오검↑ — 폭 상한(stopline_max_width_m)
        # ·min_rows·판단단 heading 게이트(decision.stopline_heading_gate)가 남는 방어선이다.
        roi_h = h if c.stopline_roi_h <= 0 else int(min(c.stopline_roi_h, h))
        y0 = h - roi_h
        roi = bev[y0:h, :]

        hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
        # 정지선색 마스크 선택(실트랙 정지선=노랑). 차선도 노랑이지만 세로라 행
        # 커버리지가 낮아 가로띠 정지선과 구분된다.
        color = str(getattr(c, "stopline_color", "yellow")).lower()
        yellow = cv2.inRange(hls, np.array(c.yellow_lo), np.array(c.yellow_hi))
        if color == "white":
            mask = self._white_mask(hls)
        elif color == "both":
            mask = cv2.bitwise_or(yellow, self._white_mask(hls))
        else:  # "yellow"(기본)
            mask = yellow

        # 방식 스위치: hough(작년 검증 가로선 길이합) or coverage(행 커버리지, 아래).
        if str(getattr(c, "stopline_method", "coverage")).lower() == "hough":
            return self._stopline_hough(mask, y0, h, w, debug)

        # 행별 정지선색 커버리지(폭 대비 비율). 정지선 행은 폭을 넓게 덮는다.
        mb = mask.astype(bool)
        row_cov = mb.sum(axis=1) / float(max(1, w))
        band = row_cov >= c.stopline_row_coverage

        # 가로폭 상한: 실측 정지선폭(stopline_max_width_m)을 넘게 '연속'으로 뻗은 행은
        # 정지선이 아님(벽/큰 얼룩/가로로 긴 노이즈)→후보 제외. 연속(run) 기준이라
        # 멀리 떨어진 별도 노랑 얼룩이 합산돼 오제외되지 않는다. 0이면 상한 없음.
        max_w_m = float(getattr(c, "stopline_max_width_m", 0.0) or 0.0)
        if max_w_m > 0.0 and c.m_per_px_lateral > 0.0:
            max_run_px = max_w_m / c.m_per_px_lateral
            for r in np.nonzero(band)[0]:
                if self._longest_run(mb[r]) > max_run_px:
                    band[r] = False

        n_band = int(band.sum())
        detected = n_band >= c.stopline_min_rows
        cov_max = float(row_cov.max()) if row_cov.size else 0.0  # 진단: 최고 행 커버리지.

        nearest_row = -1.0
        if detected:
            rows = np.nonzero(band)[0]
            nearest_row = float(int(rows.max()) + y0)  # 바닥에 가장 가까운 후보 행.

            # 디버그: 검출 밴드에 빨간 박스(노란 정지선 위에서도 잘 보이게).
            if debug is not None:
                y1 = max(0, int(rows.min()) + y0 - 4)
                y2 = min(h - 1, int(rows.max()) + y0 + 4)
                band_mask = mask[int(rows.min()):int(rows.max()) + 1, :]
                xs = np.nonzero(band_mask)[1]
                if xs.size > 0:
                    x1 = max(0, int(xs.min()) - 8)
                    x2 = min(w - 1, int(xs.max()) + 8)
                else:
                    x1, x2 = 0, w - 1
                cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 0, 255), 3)  # BGR 빨강.

        return (detected, nearest_row if detected else -1.0, cov_max, n_band)
