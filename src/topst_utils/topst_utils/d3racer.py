
from __future__ import annotations
from dataclasses import dataclass
from topst_utils.pca9685 import PCA9685

@dataclass
class ServoCalib:
    center_us: int = 1500
    span_us: int = 500          # +/- 범위 (예: 1500±500 => 1000~2000us)
    min_us: int = 1000
    max_us: int = 2000


@dataclass
class EscCalib:
    neutral_us: int = 1500
    fwd_us: int = 2000          # +1.0
    rev_us: int = 1000          # -1.0
    min_us: int = 1000
    max_us: int = 2000
    # 데드밴드 보상: ESC는 중립 위/아래로 일정 폭까지 모터가 안 돈다(죽은 구간).
    # 선형 매핑(중립부터 시작)은 이 구간에 스로틀 하위 영역을 통째로 버려 저속이
    # 안 나간다. 시작 펄스를 '모터가 실제로 도는 최소 펄스'로 잡아 그 지점부터
    # 매핑하면 작은 throttle에도 바로 구동된다. 기본=중립(=보상 off, 기존 선형 동작).
    fwd_start_us: int = 1500    # 전진 시작 펄스[µs]. 실측(바퀴가 처음 도는 값)으로 튜닝.
    rev_start_us: int = 1500    # 후진 시작 펄스[µs].


class D3Racer:
    """
    PiRacerPro와 유사한 API:
      - set_steering_percent(x): -1.0 ~ +1.0 (좌/우)
      - set_throttle_percent(x): -1.0 ~ +1.0 (후/전), 0=중립
    """
    def __init__(
        self,
        i2c_bus: int = 3,
        pca9685_addr: int = 0x40,
        freq_hz: float = 50.0,
        steering_channel: int = 0,
        throttle_channel: int = 1,
        steering: ServoCalib = ServoCalib(),
        esc: EscCalib = EscCalib(),
    ):
        self.pwm = PCA9685(bus=i2c_bus, address=pca9685_addr, freq_hz=freq_hz)
        self.st_ch = steering_channel
        self.th_ch = throttle_channel
        self.st = steering
        self.esc = esc

        # 안전: 초기 중립
        self.set_steering_percent(0.0)
        self.set_throttle_percent(0.0)

    @staticmethod
    def clip(x: float, lo: float, hi: float) -> float:
        return lo if x < lo else hi if x > hi else x

    def set_steering_percent(self, p: float):
        p = float(p)
        p = self.clip(p, -1.0, 1.0)

        pulse = self.st.center_us + p * self.st.span_us
        pulse = self.clip(pulse, self.st.min_us, self.st.max_us)
        self.pwm.set_pulse_us(self.st_ch, pulse)

    def set_throttle_percent(self, p: float):
        p = float(p)
        p = self.clip(p, -1.0, 1.0)

        # 데드밴드 보상: p>0은 fwd_start_us부터, p<0은 rev_start_us부터 매핑한다.
        # start_us=neutral_us면 기존 선형 매핑(중립부터)과 동일.
        #   p→0+ : fwd_start_us,   p=+1 : fwd_us
        #   p→0- : rev_start_us,   p=-1 : rev_us
        if p > 0:
            pulse = self.esc.fwd_start_us + p * (self.esc.fwd_us - self.esc.fwd_start_us)
        elif p < 0:
            pulse = self.esc.rev_start_us + p * (self.esc.rev_start_us - self.esc.rev_us)
        else:
            pulse = self.esc.neutral_us

        pulse = self.clip(pulse, self.esc.min_us, self.esc.max_us)
        self.pwm.set_pulse_us(self.th_ch, pulse)

    def stop(self):
        self.set_throttle_percent(0.0)

    def close(self):
        self.stop()
        self.pwm.close()
