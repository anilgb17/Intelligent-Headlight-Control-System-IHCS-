from typing import Optional
from src.models import BeamMode, Configuration, TurnSignalState

class BeamActuatorModule:
    def __init__(self, config: Configuration):
        self.config = config
        self.current_mode = BeamMode.LOW_BEAM
        self.is_blinking_active = False
        self.blinking_timer = 0.0

    def switch_beam_mode(self, mode: BeamMode):
        self.current_mode = mode

    def start_blinking(self):
        if not self.is_blinking_active:
            self.is_blinking_active = True
            self.blinking_timer = 0.0

    def stop_blinking(self):
        self.is_blinking_active = False
        self.blinking_timer = 0.0

    def update(self, dt: float):
        if self.is_blinking_active:
            self.blinking_timer += dt

    def get_current_physical_beam(self) -> BeamMode:
        """
        Returns the actual state of the beam (accounting for blinking).
        """
        if self.is_blinking_active:
            # 2Hz frequency = 0.5s period.
            # 0.0-0.25s: High? 0.25-0.5s: Low?
            # Or just follow timer logic.
            cycle_time = 1.0 / self.config.blinking_frequency
            phase = self.blinking_timer % cycle_time
            if phase < (cycle_time / 2):
                return BeamMode.HIGH_BEAM
            else:
                return BeamMode.LOW_BEAM
        else:
            return self.current_mode

    def is_blinking(self) -> bool:
        return self.is_blinking_active

class HornActuatorModule:
    def __init__(self, config: Configuration):
        self.config = config
        self.is_active = False
        self.active_timer = 0.0
        self.duration = 0.0

    def trigger_horn(self, duration: float):
        self.is_active = True
        self.active_timer = 0.0
        self.duration = duration

    def stop_horn(self):
        self.is_active = False
        self.active_timer = 0.0

    def update(self, dt: float):
        if self.is_active:
            self.active_timer += dt
            if self.active_timer >= self.duration:
                self.is_active = False
                self.active_timer = 0.0

    def is_horn_active(self) -> bool:
        return self.is_active

class TurnSignalActuatorModule:
    def __init__(self, config: Configuration):
        self.config = config
        self.state = TurnSignalState.OFF
        self.blinking_timer = 0.0
        self.is_blinking_on = False # For visual blinking effect (ON/OFF duty cycle)

    def activate_signal(self, state: TurnSignalState):
        if state != self.state:
            self.state = state
            self.blinking_timer = 0.0
            self.is_blinking_on = True

    def update(self, dt: float):
        if self.state != TurnSignalState.OFF:
            self.blinking_timer += dt
            # Simple 1Hz blinking (0.5s ON, 0.5s OFF)
            cycle_time = 1.0 # 1Hz
            phase = self.blinking_timer % cycle_time
            self.is_blinking_on = phase < 0.5

    def get_state(self) -> TurnSignalState:
        return self.state

    def is_lit(self) -> bool:
        """Returns True if the bulb is physically ON (during blinking cycle)"""
        return self.is_blinking_on and self.state != TurnSignalState.OFF
