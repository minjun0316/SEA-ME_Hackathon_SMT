"""@file config_schema.py
@brief 모든 튜닝 파라미터의 타입 안전한 스키마 + YAML 로더.

@details
"하드코딩 최소화 / 모든 파라미터는 YAML에서 수정 가능" 원칙을 강제하는
단일 지점이다. YAML은 평문이라 오타·타입 오류를 잡지 못하므로,
dataclass로 스키마를 정의하고 from_dict()에서 검증/기본값 처리를 한다.

@par 설계 이유
- 컨트롤러 클래스는 dict가 아니라 타입이 있는 Config 객체를 받는다 →
  IDE 자동완성, 오타 즉시 발견, 단위 테스트에서 객체 직접 생성 용이.
- ROS2 노드에서도 동일 dataclass를 ROS 파라미터로부터 채워 재사용한다
  (시뮬↔실차 코드 일원화).
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Any, Dict

import yaml


def _filter_known(cls, data: Dict[str, Any]) -> Dict[str, Any]:
    """@brief dataclass가 정의한 필드만 추려 알 수 없는 키를 무시한다."""
    known = {f.name for f in fields(cls)}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"{cls.__name__}: unknown config keys {sorted(unknown)}")
    return {k: v for k, v in data.items() if k in known}


@dataclass
class VehicleConfig:
    """@brief 차량 물리/액추에이터 파라미터."""

    wheelbase: float = 0.26          ##< 휠베이스 L [m] (스케일카 기준 예시).
    max_steer_deg: float = 24.0      ##< 물리 조향각 한계 [deg].
    steer_trim: float = 0.0          ##< 직진 보정(중립 오프셋), 정규화 단위.
    speed_tau: float = 0.15          ##< 속도 1차 지연 시상수 [s] (시뮬용).

    @property
    def max_steer_rad(self) -> float:
        """@brief 조향각 한계를 라디안으로 반환."""
        import math
        return math.radians(self.max_steer_deg)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "VehicleConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class PurePursuitConfig:
    """@brief Pure Pursuit 조향 파라미터.

    @details Adaptive Lookahead: @f$ L_d = L_{min} + K_v v - K_c \\kappa @f$
    """

    use_adaptive_lookahead: bool = True  ##< False면 fixed_lookahead 사용(초기 테스트).
    fixed_lookahead: float = 0.6         ##< 고정 lookahead [m].
    ld_min: float = 0.3                  ##< Adaptive 최소 lookahead [m].
    ld_max: float = 1.2                  ##< Adaptive 최대 lookahead [m].
    kv: float = 0.4                      ##< 속도 이득 K_v [s].
    kc: float = 0.1                      ##< 곡률 이득 K_c [m^2].
    curvature_preview: float = 0.0       ##< adaptive/gain용 곡률을 nearest 대신 전방 preview[m] 구간의 max|κ|로 읽는다. 0이면 nearest만. >0이면 곡선 진입 전 Ld가 미리 수축→턴인 지연(직선 Ld는 무영향).
    steering_gain: float = 1.0           ##< Pure Pursuit 출력 스케일 보정(곡률 스케줄 시 직선 baseline=하한).
    steering_smoothing: float = 0.3      ##< 신규 명령 가중치 β: out = (1-β)·old + β·new.
    search_window: int = 60              ##< 전방 최근접 탐색 윈도우(점 개수). 0이면 전역 탐색.
    # 곡률 스케줄 gain: gain = clip(steering_gain + kappa*κ_eff, steering_gain, max).
    use_curvature_gain: bool = False     ##< True면 곡률에 따라 gain 가산(직선 낮게/커브 높게). False=상수.
    steering_gain_kappa: float = 0.0     ##< |κ_eff| 계수. +면 커브서 gain↑.
    steering_gain_max: float = 1.2       ##< 곡률 가산 후 gain 상한(클램프).
    curvature_deadband: float = 0.3      ##< 이하 |κ|는 직선 취급(노이즈 격리). κ_eff=max(0,|κ_s|-deadband).
    curvature_smoothing: float = 0.3     ##< 곡률 EMA 가중치: κ_s = (1-α)·κ_s + α·|κ|. 작을수록 강한 저역통과.

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PurePursuitConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class LateralPDConfig:
    """@brief 근거리 차선오차 PD(+heading) 횡제어 파라미터.

    @details δ = k_ff·κ + k_heading·heading_error + k_cross·lateral_offset + k_deriv·d(e)/dt.
    부호 규약: +오차/+heading/+κ(좌커브) → +조향(좌). 출력은 정규화[-1,1]+트림+smoothing.
    """

    steering_sign: float = 1.0  ##< 조향 부호(차량 적응). +1=+오차→좌회전. 거치대서 반대로 꺾이면 -1. ⚠️실차 첫 구동 전 확인.
    k_cross: float = 1.2        ##< lateral_offset[m] → 정규화 조향(직선 baseline). 예: 0.1m→0.12. 차선복귀 P.
    k_cross_kappa: float = 0.0  ##< |κ_eff| 당 k_cross 가산. 커브서 중심복귀 부스트, 직선(κ_eff=0)은 baseline. 0=스케줄 off(상수 k_cross).
    k_cross_max: float = 1.5    ##< 곡률 스케줄된 k_cross 상한(과조향 방지).
    k_heading: float = 0.8      ##< heading_error[rad] → 조향. 위빙 억제·자연 감쇠(Stanley heading항). 커브 못돌면 ↑.
    k_deriv: float = 0.0        ##< d(lateral)/dt 항(선택 D). 근거리 노이즈 커서 기본 0(heading이 주 감쇠).
    deriv_smoothing: float = 0.3  ##< 미분 EMA 가중치(노이즈 저역통과). 작을수록 강한 필터.
    max_offset: float = 0.5     ##< lateral_offset 클램프[m](오검출 스파이크→과조향 방지).
    steering_smoothing: float = 0.3  ##< 출력 β: out=(1-β)·old+β·new. 작을수록 부드럽(지연↑).
    # --- 곡률 피드포워드(신규): 커브 유지 조향을 오차 없이 미리 얹어 PD의 정상상태
    #     오차(커브서 중심 못잡음)를 제거. κ는 lane_path 근거리에서 유도(부호 있음). ---
    k_ff: float = 0.0           ##< 부호곡률 κ[1/m] → 조향 피드포워드 이득. 0=off(기존 순수PD). 커브서 안쪽 못붙으면 ↑.
    curvature_smoothing: float = 0.3  ##< κ EMA 가중치(lane_path 노이즈 저역통과). 작을수록 강한 필터·지연↑.
    curvature_deadband: float = 0.0   ##< 이하 |κ_s|는 직선 취급(ff=0). 직선 곡률노이즈 격리. 직선서 ff 들썩이면 ↑.
    curvature_preview: float = 0.4    ##< 근거리 부호곡률 평균 구간[m]. 차량 앞 이 거리의 평균 κ를 피드포워드로.

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "LateralPDConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class SpeedConfig:
    """@brief 속도 결정(곡률 기반) + Speed PID 파라미터."""

    v_max: float = 0.8               ##< 직선 최대 속도 [정규화 또는 m/s].
    v_min: float = 0.3               ##< 급커브 최소 속도.
    curvature_gain: float = 0.5      ##< 곡률→감속 이득 (|κ|에 비례 감속).
    use_speed_pid: bool = False      ##< True면 목표속도→throttle을 PID로 산출.
    kp: float = 0.5                  ##< Speed PID 비례 이득.
    ki: float = 0.0                  ##< Speed PID 적분 이득.
    kd: float = 0.0                  ##< Speed PID 미분 이득.

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SpeedConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class StoplineManeuverConfig:
    """@brief 정지선 카운트 → 개루프 고정스티어 기동 파라미터(로터리 진입/탈출).

    @details on/off는 운영 플래그라 controller_node의 ROS 파라미터
    `stopline_maneuver_enable`(런치 인자)로 켠다. 여기 값들은 튜닝값이다.
    조향 크기는 **트림 전 raw**[-1,1]: +면 좌, -면 우. @see core/planning/stopline_maneuver.py
    """

    steer: float = 0.35        ##< 고정 조향 크기(정규화, 트림 전 raw). "조금". 실차서 커브 못 돌면 ↑, 과회전이면 ↓.
    duration_sec: float = 1.5  ##< 기동 유지 시간[s]. 회전이 덜 되면 ↑, 지나치면 ↓.
    first_dir: float = 1.0     ##< 1번째 정지선 방향(+1=좌회전).
    second_dir: float = -1.0   ##< 2번째 정지선 방향(-1=우회전, 탈출).
    debounce_sec: float = 1.5  ##< 정지선 카운트 최소 간격[s](한 정지선 중복 카운트 방지).
    max_count: int = 2         ##< 이 카운트까지만 기동(이후 정지선 무시).

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "StoplineManeuverConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class DecisionConfig:
    """@brief 판단(State Machine) 임계값/배율 파라미터.

    @details 판단은 제어기 튜닝값(controller.yaml)을 건드리지 않고 상태별
    "배율/게이트"만 정한다(계약 docs/interfaces.md §4.4). 여기의 값들은 상태
    전이 임계값과 각 상태에서 내보낼 speed_scale/lookahead_scale이다.
    시간 임계값은 초[s] 단위이며 update(obs, dt)의 dt로 누적된다.
    """

    # --- 유효 검출 판정 ---
    min_points: int = 2              ##< lane_path 최소 점 개수(미만이면 미검출 취급).
    conf_min: float = 0.35           ##< 이 신뢰도 미만이면 미검출 취급(→ grace 후 LOST).
    conf_drive: float = 0.60         ##< 이 이상이면 정상 DRIVE 후보. 미만이면 SLOW.
    # --- 소실/복귀 타이머 ---
    lost_grace: float = 0.30         ##< 미검출이 이 시간 이상 지속되면 LOST[s].
    recover_grace: float = 0.10      ##< INIT/LOST 탈출에 필요한 연속 유효검출 시간[s].
    # --- 감속(SLOW) 유발 조건 ---
    heading_slow: float = 0.35       ##< |heading_error|가 이 값 이상이면 SLOW[rad](~20°).
    offset_slow: float = 0.12        ##< |lateral_offset|가 이 값 이상이면 SLOW[m].
    # --- 정지선 ---
    stop_trigger_dist: float = 0.25  ##< 정지선이 이 거리 이내면 STOP[m].
    stop_approach_dist: float = 0.60 ##< 정지선이 이 거리 이내면 접근 감속 SLOW[m].
    stop_dwell: float = 2.0          ##< STOP 유지 시간[s]. 이후 정지선 유지돼도 재출발.
    stopline_heading_gate: float = 0.0  ##< 커브 게이트[rad]: |heading_error|가 이 값 이상이면 정지선 무시(커브서 가로로 눕는 차선 오인 차단). 0=게이트 없음. heading_slow(0.35)보다 약간 크게 ~0.40 권장.
    # --- 상태별 출력 배율 ---
    drive_speed_scale: float = 1.0   ##< DRIVE 속도 배율.
    slow_speed_scale: float = 0.5    ##< SLOW 속도 배율.
    slow_lookahead_scale: float = 0.8 ##< SLOW lookahead 배율(코너서 가까이).

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "DecisionConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class MissionConfig:
    """@brief 상위 미션 시퀀스(MissionSequencer) 파라미터.

    @details 미션 SM(WAIT_START_SIGNAL~FINISH_STOP)의 구간별 배율. 로터리 회전은
    제어단 StoplineManeuver가 담당하므로 로터리 타이머/방향은 여기 없다(그쪽 config는
    StoplineManeuverConfig). 자세한 시퀀스는 docs/mission_fsm.md 참조.
    """

    # --- 구간별 배율 ---
    slow_speed_scale: float = 0.5            ##< 장애물 구역 등 감속 구간 속도 상한 배율.
    # --- YOLO 추론 게이트 ---
    yolo_gate_enable: bool = True            ##< True=페이즈별 YOLO on/off(출발·도착만 ON, 주행중 OFF로 FPS 확보). False=게이트 끔(항상 ON=기존 동작).
    # --- 출발 신호등 게이트 (07-15 밤, 테스트용) ---
    # False면 WAIT_START_SIGNAL을 건너뛰고 LANE_FOLLOW에서 시작한다 = 초록불 없이 즉시 출발.
    # 팻말/차선만 따로 검증할 때 신호등을 준비하지 않아도 되게 하는 **테스트 스위치**다.
    # ⚠ 대회 주행은 반드시 True(출발 신호 준수). decision_node의 동명 파라미터와 같은 의미.
    traffic_light_start_enable: bool = True  ##< False=초록불 대기 건너뛰고 즉시 주행 시작(테스트용).
    # --- 방향 팻말 분기(SIGN_BRANCH) — 07-15 밤 재설계 ---
    # 팻말 YOLO가 좌/우를 conf≥sign_min_conf로 읽으면 진입 → 방향 래치 → turn_hint로
    # 인지에 앵커 차선을 지시(그쪽 선 하나만 추종 = 폐루프). 고정 조향 bias는 폐기.
    sign_branch_duration: float = 5.0        ##< SIGN_BRANCH 지속 **상한**[s](경과 시 LANE_FOLLOW 복귀).
    sign_branch_speed_scale: float = 0.5     ##< 팻말 분기 중 속도 상한 배율(감속).
    # [07-16] 팻말이 시야에서 사라지고 이만큼 지나면 SIGN_BRANCH 종료(= offset 해제).
    # 왜 필요한가: offset은 '도로 한가운데 팻말'을 비켜 지나는 용도인데, 차선폭이 36.6cm
    # (=중심→선 18.3cm)뿐이라 9cm offset을 팻말을 지난 뒤에도 물고 있으면 바깥 바퀴와
    # 흰선 사이 여유가 1.8cm밖에 안 남아 **차선을 못 따라가고 이탈한다**(07-16 실차).
    # 고정 타이머로는 못 푼다 — 통과 시점은 속도에 달렸고, 그 속도는 배터리에 민감하다
    # (sign_branch_speed_scale=0.2 = 데드밴드 문턱 위 17µs). 팻말이 **시야에서 사라진 것**이
    # 곧 '거의 다 왔다'는 물리적 사건이므로 그걸 기준으로 삼는다.
    # 0으로 두면 옛 동작(고정시간만). 팻말을 지나기 전에 풀려 들이받으면 ↑,
    # 지나고도 한참 치우쳐 달리면 ↓.
    sign_lost_release_sec: float = 1.0       ##< 팻말 무관측 이만큼 지속 시 SIGN_BRANCH 종료. 0=끔.
    # [07-16] 분기 종료 후 속도 복귀 램프[s]. 0=끔(계단식 복귀, 옛 동작).
    # 왜: 종료 순간 speed_scale이 분기값 → 1.0으로 **한 프레임에** 뛴다. 스로틀은 데드밴드
    # 문턱 바로 위의 가파른 구간이라(1600µs → 1660µs = 문턱 위 여유 26 → 85µs, 3배 이상)
    # 체감 가속이 계단이고, 하필 그 지점이 갈래가 **다시 합쳐지는 구간**이라 제어가 불안정
    # 해졌다(07-16 실차). 속도를 sign_branch_speed_scale → 1.0으로 이 시간에 걸쳐 선형 복귀.
    # 합류가 끝나도록 넉넉히 잡되, 길면 그만큼 코스 시간을 잃는다. 여전히 급하면 ↑.
    sign_exit_ramp_sec: float = 1.5          ##< 분기 종료 후 속도 복귀에 쓰는 시간[s]. 0=끔.
    # [07-16b] 램프에 **곡선 게이트**를 건다. 시계로만 도는 램프는 갈래가 다시 합쳐지는
    # 곡선을 언제 빠져나오는지 모른다 — 통과 시점은 속도에, 그 속도는 배터리에 달렸다
    # (sign_lost_release_sec과 같은 함정). 고정 1.5s가 곡선 한복판에서 만료돼 거기서
    # 이미 속도가 붙었고 꿀렁임이 심해졌다(07-16 실차). → |heading_error|가 이 값 미만
    # (=차선이 폈다)일 때만 램프 타이머를 흘린다. 곡선 중엔 얼려서 분기 속도를 유지한다.
    # heading_slow(0.35)보다 **낮게** 잡아야 '확실히 편 뒤에만' 속도를 올린다.
    # 0=게이트 끔(옛 동작=순수 시간 램프).
    # ⚠ 노이즈는 안전한 쪽으로 실패한다(램프가 잠깐 멈출 뿐) — 그래서 중앙값 필터가 없다.
    # 곡선에서 아직 속도가 붙으면 ↑, 직선인데 복귀가 늦으면 ↓.
    sign_exit_straight_rad: float = 0.20     ##< 램프 진행 조건: |heading_error| 이 값 미만 = 직선[rad]. 0=게이트 끔.
    # 위 게이트의 **폴백 상한**. heading_error가 노이즈/편향으로 계속 문턱 위에 머물면
    # 램프가 영영 안 흘러 코스 끝까지 분기 속도로 기어간다 → 곡선에 갇힌 시간이 이 값을
    # 넘으면 게이트를 무시하고 램프를 진행시킨다. sign_branch_duration이 상한 폴백으로
    # 남은 것과 같은 패턴이다. 합류 곡선을 넉넉히 덮되, 길면 그만큼 코스 시간을 잃는다.
    sign_exit_hold_max_sec: float = 4.0      ##< 곡선 게이트로 램프를 붙들 수 있는 최대 시간[s]. 초과 시 게이트 무시.
    # [07-16b] 팻말 분기를 마치면 제어에 post_sign **게인 프로파일**을 지시할지.
    # 실제 게인 수치는 controller.yaml의 lateral_pd_post_sign 블록에 있다(튜닝 일원화,
    # 계약 §4.4) — 판단은 "어느 구간인지"만 말하고 값은 모른다. False=끔(전 구간 baseline).
    post_sign_profile_enable: bool = True    ##< 팻말 분기 후 post_sign 게인 프로파일 지시. False=끔.
    # --- 동적 장애물(아루코) — 07-15 ---
    # 아루코가 사라졌을 때 '진짜 재출발'과 '오검출이었음'을 가르는 최소 체류시간.
    # 이보다 짧게 머물렀으면 오검출로 보고 FINISH_WATCH가 아니라 LANE_FOLLOW로 되돌린다
    # (오검출 1프레임이 팻말 분기를 통째로 스킵하고 첫 빨간불에 코스를 끝내던 것을 막음).
    # ⚠ 인지의 aruco_hold_sec보다 커야 의미가 있다: hold가 present를 늘려주므로 실효
    #   요구치는 (이 값 - hold_sec) = '심판이 마커를 실제로 들고 있어야 하는 시간'이다.
    #   현재 hold=1.0 + 이 값 2.0 → 실효 1.0s. 심판이 더 짧게 보여주면 ↓.
    obstacle_min_dwell_sec: float = 2.0      ##< OBSTACLE_ZONE 최소 체류[s]. 미달 시 오검출로 보고 LANE_FOLLOW 복귀.
    # --- 빨간불 정지 해제(FINISH_STOP → FINISH_WATCH) — 07-15c ---
    # FINISH_STOP은 원래 편도(전이 없음)라 빨강 오검출 하나가 코스를 복구 불가로 끝냈다.
    # 대회 중엔 동글을 빼므로 페이즈를 손으로 되돌릴 수도 없어 그대로 미션 실패였다.
    # → 마지막으로 빨강을 본 지 이 시간이 지나면 FINISH_WATCH로 복귀해 주행을 재개한다.
    # 이 값이 곧 **최소 정지시간**이기도 하다: FINISH_STOP 진입은 '방금 빨강을 봤다'는
    # 뜻이라 무관측 타이머가 0에서 시작하므로, 아무리 빨라도 이 시간은 서 있는다
    # (= 심판에게 '정지했음'을 보여주는 증명. 정지 후 재출발 자체엔 패널티가 없다).
    # 세 경우 모두 이 조건 하나로 처리된다:
    #   진짜 빨강(심판이 계속 들고 있음) → 타이머가 계속 0으로 리셋 → 영구 정지(정상 종료).
    #   오검출 → 빨강이 사라짐 → 복귀 → 미션 계속 → 진짜 빨강에서 다시 정지(코스 완주).
    #   오버슛(정지했으나 빨강이 화면 밖) → 복귀. 증명은 이미 끝났으므로 손해 없음.
    # ⚠ 0=끔이면 옛 동작(영구 정지). 오검출 복구가 불가능해지므로 대회 주행은 >0 권장.
    # ⚠ 못 고치는 케이스: 오검출 원인이 시야 안 '정적인' 물체면 멈춘 차의 시야가 얼어붙어
    #   계속 검출된다 → 영구 정지. 그건 인지의 red_min_box_h_frac(거리 게이트) 몫이다.
    finish_stop_release_sec: float = 10.0    ##< 빨강 마지막 관측 후 이 시간 지나면 FINISH_WATCH 복귀(=최소 정지시간). 0=끔(영구 정지).

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MissionConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class BatteryConfig:
    """@brief 배터리 전압 보정 파라미터."""

    enabled: bool = False            ##< 배터리 토픽이 있을 때만 True.
    v_nominal: float = 7.4           ##< 기준(공칭) 전압 [V] (2S LiPo 예시).
    v_min: float = 6.4               ##< 보정 하한 전압(이하 클램프) [V].
    v_max: float = 8.4               ##< 보정 상한 전압(이상 클램프) [V].
    max_gain: float = 1.5            ##< 보정 배수 상한 (과보상 방지).
    # 선택적 룩업 테이블: [[voltage, gain], ...]. 비어 있으면 수식 보정 사용.
    lookup_table: list = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "BatteryConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class SimConfig:
    """@brief 시뮬레이션 실행 파라미터."""

    dt: float = 0.02                 ##< 적분 시간 간격 [s] (50 Hz).
    max_time: float = 60.0           ##< 최대 시뮬 시간 [s].
    goal_tolerance: float = 0.15     ##< 경로 끝 도달 판정 거리 [m].
    path_name: str = "s_curve"       ##< 사용할 정적 경로 이름.
    battery_voltage: float = 7.4     ##< 시뮬 중 가정 배터리 전압 [V].

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SimConfig":
        return cls(**_filter_known(cls, data or {}))


@dataclass
class AppConfig:
    """@brief 전체 설정 묶음 (여러 YAML을 합쳐 구성)."""

    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    pure_pursuit: PurePursuitConfig = field(default_factory=PurePursuitConfig)
    lateral_pd: LateralPDConfig = field(default_factory=LateralPDConfig)
    # [07-16b] 팻말 분기 후 구간(직선-ㄱ자-직선-ㄱ자) 전용 lateral_pd 게인 세트.
    # baseline(lateral_pd)을 상속하고 yaml의 lateral_pd_post_sign에 적은 키만 덮어쓴다
    # → 그 구간 값을 아무리 바꿔도 msg/계약 변경이 없다(판단은 프로파일 id만 지시).
    # 왜 세트인가: S자는 곡률이 연속이라 선반영이 이득이고, 팻말 뒤는 직선↔90도 계단이라
    # 같은 게인으로 둘 다 만족시킬 수 없다(k_heading 0.5→0.33 싸움이 그 증거).
    lateral_pd_post_sign: LateralPDConfig = field(default_factory=LateralPDConfig)
    speed: SpeedConfig = field(default_factory=SpeedConfig)
    stopline_maneuver: StoplineManeuverConfig = field(default_factory=StoplineManeuverConfig)
    decision: DecisionConfig = field(default_factory=DecisionConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    sim: SimConfig = field(default_factory=SimConfig)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "AppConfig":
        data = data or {}
        # post_sign 프로파일은 baseline을 **상속**한다: lateral_pd에 적힌 값을 깔고
        # lateral_pd_post_sign에 적힌 키만 덮어쓴다. 그래야 baseline을 튜닝했을 때
        # 그 구간만 조용히 옛 값에 남는 일이 없다(두 블록을 따로 관리하면 반드시 갈라진다).
        lpd_base = data.get("lateral_pd", {}) or {}
        lpd_post = {**lpd_base, **(data.get("lateral_pd_post_sign", {}) or {})}
        return cls(
            vehicle=VehicleConfig.from_dict(data.get("vehicle", {})),
            pure_pursuit=PurePursuitConfig.from_dict(data.get("pure_pursuit", {})),
            lateral_pd=LateralPDConfig.from_dict(lpd_base),
            lateral_pd_post_sign=LateralPDConfig.from_dict(lpd_post),
            speed=SpeedConfig.from_dict(data.get("speed", {})),
            stopline_maneuver=StoplineManeuverConfig.from_dict(data.get("stopline_maneuver", {})),
            decision=DecisionConfig.from_dict(data.get("decision", {})),
            mission=MissionConfig.from_dict(data.get("mission", {})),
            battery=BatteryConfig.from_dict(data.get("battery", {})),
            sim=SimConfig.from_dict(data.get("sim", {})),
        )


def load_config(*yaml_paths: str) -> AppConfig:
    """@brief 하나 이상의 YAML 파일을 순서대로 병합해 AppConfig를 만든다.

    @param yaml_paths 읽을 YAML 경로들. 뒤쪽 파일이 앞쪽을 덮어쓴다(merge).
    @return 검증된 AppConfig.

    @note 최상위 키(vehicle/pure_pursuit/lateral_pd/speed/stopline_maneuver/decision/mission/battery/sim) 단위로 얕게
          병합한다. 같은 섹션의 일부 키만 override 하려면 해당 섹션 전체를
          한 파일에 두는 것을 권장한다.
    """
    merged: Dict[str, Any] = {}
    for path in yaml_paths:
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        for section, values in data.items():
            merged.setdefault(section, {})
            if isinstance(values, dict):
                merged[section].update(values)
            else:
                merged[section] = values
    return AppConfig.from_dict(merged)
