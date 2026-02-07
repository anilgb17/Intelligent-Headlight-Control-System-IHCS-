from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

class BeamMode(Enum):
    HIGH_BEAM = auto()
    LOW_BEAM = auto()

class OvertakingStatus(Enum):
    NONE = auto()
    IN_PROGRESS = auto()
    COMPLETE = auto()
    ABORTED = auto()

class VehicleType(Enum):
    ONCOMING = auto()
    AHEAD = auto()

class SystemState(Enum):
    NORMAL = auto()
    ERROR = auto()
    MANUAL_OVERRIDE = auto()

class TurnSignalState(Enum):
    OFF = auto()
    LEFT = auto()
    RIGHT = auto()
    HAZARD = auto()

class EngineState(Enum):
    OFF = auto()
    RUNNING = auto()
    STALLED = auto()

class HazardType(Enum):
    NONE = auto()
    BLOCKING_VEHICLE = auto() # Oncoming or overtaking vehicle blocking path
    STALLED_VEHICLE = auto() # Ego vehicle stalled

@dataclass(frozen=True)
class SteeringMeasurement:
    angle: float # Degrees. +Left, -Right
    timestamp: float

@dataclass(frozen=True)
class YawRate:
    rate: float # Degrees/second. +Left, -Right
    timestamp: float

@dataclass(frozen=True)
class Position:
    x: float  # Lateral position (meters)
    y: float  # Longitudinal position (meters)
    
    def is_valid(self) -> bool:
        return isinstance(self.x, (int, float)) and isinstance(self.y, (int, float))

@dataclass(frozen=True)
class Velocity:
    vx: float # Lateral velocity (m/s)
    vy: float # Longitudinal velocity (m/s)

    def is_valid(self) -> bool:
        return isinstance(self.vx, (int, float)) and isinstance(self.vy, (int, float))

@dataclass
class DetectedVehicle:
    id: int
    type: VehicleType
    position: Position
    velocity: Velocity
    distance: float

    def is_valid(self) -> bool:
        if not isinstance(self.id, int) or self.id < 0:
            return False
        if not self.position.is_valid():
            return False
        if not self.velocity.is_valid():
            return False
        if not isinstance(self.distance, (int, float)) or self.distance < 0:
            return False
        return True

@dataclass
class Configuration:
    detection_range: float = 200.0  # meters
    safe_distance: float = 50.0    # meters
    transition_time_limit: float = 0.2  # seconds
    update_frequency: float = 10.0      # Hz
    blinking_frequency: float = 2.0     # Hz
    horn_pulse_duration_min: float = 0.2 # seconds
    horn_pulse_duration_max: float = 0.3 # seconds
