from dataclasses import dataclass
from statistics import fmean
from collections import deque

class RMSTrack:
    def __init__(self, window_size: int):
        self.window_size = window_size
        self.values = deque([0]*window_size, maxlen=window_size)
        self.current_rms = 0.0
    
    def add_point(self, value: float):
        self.values.append(value**2)
        self.current_rms = fmean(self.values) ** 0.5

class EMATrack:
    def __init__(self, smoothing: float, window: int):
        self.smoothing = smoothing
        self.current_value = 0.0
        self.window = window

        self.A = (self.smoothing/(1+self.window))
        self.B = (1 - (self.smoothing/(1+self.window)))

    def add_point(self, value: float):
        self.current_value = value * self.A + self.current_value * self.B

@dataclass
class ActuatorState:
    id: int = -1 # -1 indicates unknown ID (let the low level decide)
    pos_cmd: float = 0.0
    vel_cmd: float = 0.0
    trq_cmd: float = 0.0
    kp: float = 0.0
    kd: float = 0.0

    pos_state: float = 0.0
    vel_state: float = 0.0
    trq_state: float = 0.0

    actuation_enabled: bool = False

    time: float = -1.0

    def update(self, d: dict):
        self.__dict__.update(d)

@dataclass
class IMUState:
    accx: float = 0.0
    accy: float = 0.0
    accz: float = 0.0
    gyrox: float = 0.0
    gyroy: float = 0.0
    gyroz: float = 0.0
    magx: float = 0.0
    magy: float = 0.0
    magz: float = 0.0
    orient_w: float = 1.0
    orient_x: float = 0.0
    orient_y: float = 0.0
    orient_z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def update(self, d: dict):
        self.__dict__.update(d)

@dataclass
class ControllerState:
    kp_stance: float = 0.0
    tolerance_stance: float = 0.0
    stance_angle: float = 0.0

    kp_swing: float = 0.0
    tolerance_swing: float = 0.0
    swing_angle: float = 0.0

    kd: float = 0.0

    vsync: bool = False

    knee_angle: float = 0.0
    target_angle: float = 0.0
    error: float = 0.0
    trq_cmd: float = 0.0
    calibration_offset: float = 0.0

    def update(self, d: dict):
        self.__dict__.update(d)

@dataclass
class GaitState:
    gait_phase: float = 0.0 # Ranges from 0.0 to 1.0 (aka 0% to 100%). Continuous.
    gait_state: int = -1 # IDLE = -1, STANCE = 1, SWING = 0. Discrete.
    gait_idx: int = 0 # Counts the number of gait cycles completed. Discrete.
    mean_gait_duration: float = 1.0 # The average duration of a full gait cycle in seconds. Continuous.

    def update(self, d: dict):
        self.__dict__.update(d)