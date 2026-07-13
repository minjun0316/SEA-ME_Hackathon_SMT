"""@file test_stopline_maneuver.py
@brief 정지선 카운트 → 개루프 고정스티어 기동 상태기계 단위 테스트(ROS-free).
"""
from core.config_schema import StoplineManeuverConfig
from core.planning.stopline_maneuver import StoplineManeuver


def _cfg(**kw):
    base = dict(steer=0.4, duration_sec=1.0, first_dir=1.0, second_dir=-1.0,
                debounce_sec=1.5, max_count=2)
    base.update(kw)
    return StoplineManeuverConfig(**base)


def test_idle_returns_none():
    m = StoplineManeuver(_cfg())
    assert m.update(False, 0.1) is None
    assert m.active is False
    assert m.count == 0


def test_first_stopline_triggers_left():
    m = StoplineManeuver(_cfg())
    out = m.update(True, 0.1)          # rising edge → 1번째 카운트.
    assert out is not None and out > 0  # 좌(+).
    assert m.count == 1 and m.active


def test_maneuver_runs_full_duration_open_loop():
    """활성 기동은 stop_line=False(차선 소실)여도 duration 동안 고정 조향 유지."""
    dt = 0.1
    m = StoplineManeuver(_cfg(duration_sec=1.0, steer=0.4))
    assert m.update(True, dt) == 0.4    # 시작 틱.
    steers = []
    for _ in range(20):                 # 이후 stop_line 사라져도 계속.
        steers.append(m.update(False, dt))
    active = [s for s in steers if s is not None]
    # 약 1.0s(±1틱) 동안 0.4 유지 후 None.
    assert all(s == 0.4 for s in active)
    assert 8 <= len(active) <= 11
    assert m.active is False            # 종료.


def test_debounce_blocks_recount_of_same_line():
    """기동 종료 직후 같은 정지선(계속 True)이 재카운트되지 않음(debounce)."""
    dt = 0.1
    m = StoplineManeuver(_cfg(duration_sec=0.3, debounce_sec=1.5))
    m.update(True, dt)                  # 1번째 카운트.
    for _ in range(5):                  # 기동 소모(0.3s).
        m.update(True, dt)             # stop_line 계속 True로 붙어 있어도
    assert m.count == 1                 # 종료 직후엔 아직 2로 안 셈.


def test_second_stopline_triggers_right_after_debounce():
    dt = 0.1
    m = StoplineManeuver(_cfg(duration_sec=0.3, debounce_sec=0.5, steer=0.4))
    # 1번째.
    m.update(True, dt)
    while m.active:
        m.update(False, dt)
    assert m.count == 1
    # debounce 경과(선 사라진 채 대기).
    for _ in range(7):
        m.update(False, dt)
    # 2번째 정지선(rising).
    out = m.update(True, dt)
    assert out is not None and out < 0  # 우(-).
    assert m.count == 2


def test_max_count_ignores_third():
    dt = 0.1
    m = StoplineManeuver(_cfg(duration_sec=0.2, debounce_sec=0.3, max_count=2))

    def _one_stopline():
        m.update(True, dt)
        while m.active:
            m.update(False, dt)
        for _ in range(5):             # debounce 경과.
            m.update(False, dt)

    _one_stopline()   # count 1
    _one_stopline()   # count 2
    # 3번째 시도.
    out = m.update(True, dt)
    assert out is None
    assert m.count == 2


def test_reset():
    m = StoplineManeuver(_cfg())
    m.update(True, 0.1)
    m.reset()
    assert m.count == 0 and m.active is False
