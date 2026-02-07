from typing import List, Optional
import math

from src.models import (
    Configuration, DetectedVehicle, Position, Velocity, 
    BeamMode, OvertakingStatus, SystemState, VehicleType, TurnSignalState
)
from src.vehicle_detection import VehicleDetectionModule
from src.overtaking_detection import OvertakingDetectionModule
from src.decision_engine import DecisionEngine
from src.actuators import BeamActuatorModule, HornActuatorModule, TurnSignalActuatorModule
from src.error_handling import ErrorHandler

class HeadlightController:
    def __init__(
        self, 
        config: Configuration,
        vehicle_detection: VehicleDetectionModule,
        overtaking_detection: OvertakingDetectionModule,
        decision_engine: DecisionEngine,
        beam_actuator: BeamActuatorModule,
        horn_actuator: HornActuatorModule,
        turn_signal_actuator: TurnSignalActuatorModule,
        error_handler: ErrorHandler = None 
    ):
        self.config = config
        self.vehicle_detection = vehicle_detection
        self.overtaking_detection = overtaking_detection
        self.decision_engine = decision_engine
        self.beam_actuator = beam_actuator
        self.horn_actuator = horn_actuator
        self.turn_signal_actuator = turn_signal_actuator
        self.error_handler = error_handler if error_handler else ErrorHandler()
        
        self.state = SystemState.NORMAL
        self.last_blink_cycle = -1
        self.prev_ego_position: Optional[Position] = None

    def process_update(
        self, 
        ego_position: Position, 
        raw_sensor_data: List[DetectedVehicle], 
        dt: float,
        steering_angle: float = 0.0,
        yaw_rate: float = 0.0,
        ego_velocity: Velocity = Velocity(0, 0), # Added explicit velocity
        engine_rpm: float = 2000.0 # Added RPM
    ):
        """
        Main control loop execution.
        """
        # Calculate Lateral Velocity for Turn Signals
        lateral_velocity = 0.0
        if self.prev_ego_position and dt > 0:
            lateral_velocity = (ego_position.x - self.prev_ego_position.x) / dt
        self.prev_ego_position = ego_position

        # Manual Override Check
        if self.state == SystemState.MANUAL_OVERRIDE:
             try:
                self.vehicle_detection.process_sensor_data(raw_sensor_data)
                self.overtaking_detection.process_update(ego_position, list(self.vehicle_detection.tracked_vehicles.values()))
             except Exception as e:
                self.error_handler.log_error(f"Error in manual tracking: {str(e)}", 100)
             return

        try:
            # Check for active error state
            if self.state == SystemState.ERROR:
                self.beam_actuator.switch_beam_mode(BeamMode.LOW_BEAM)
                pass
            
            # --- 0. STALL PROTECTION (OBD-II) ---
            # If moving but RPM is 0 -> Stall?
            # Or if dropped suddenly? 
            # Simple logic: If RPM < 300 and Speed > 5 m/s -> Stall while moving.
            speed = math.sqrt(ego_velocity.vx**2 + ego_velocity.vy**2)
            if engine_rpm < 300 and speed > 1.0:
                 # ENGINE FAILED
                 # Action: Activate Hazard Lights immediately
                 self.turn_signal_actuator.activate_signal(TurnSignalState.HAZARD)
                 self.error_handler.log_error("ENGINE STALL DETECTED! Hazards Activated.", 50)
                 # Maintain beams? Maybe Low Beam for safety?
                 self.beam_actuator.switch_beam_mode(BeamMode.LOW_BEAM)
                 # Skip other logic? Yes, stall is critical.
                 # But we might still want to honk if collision imminent?
                 # Let's check hazards below but prioritize Stall reaction on lights.
            
            
            # 1. Process Sensor Data
            tracked_vehicles = self.vehicle_detection.process_sensor_data(raw_sensor_data)
            
            # --- 1.5 HAZARD DETECTION (Radar/LiDAR) ---
            # Check for immediate collision risks
            from src.models import HazardType
            hazard_type, hazard_id, msg = self.vehicle_detection.detect_hazards(ego_velocity)
            
            if hazard_type == HazardType.BLOCKING_VEHICLE:
                # ACTION: RAPID HIGH BEAM PULSE + HORN
                self.error_handler.log_error(f"HAZARD: {msg}", 10) # Log as warning
                
                # Rapid Pulse (High/Low toggling faster than normal 2Hz)
                # We need a new state or just force it here?
                # Actuator supports `start_blinking`. We can change frequency in Config or manually toggle.
                # Let's manually force High Beam and Horn.
                
                # Override everything
                self.beam_actuator.switch_beam_mode(BeamMode.HIGH_BEAM) # Pulse ON
                self.horn_actuator.trigger_horn(0.5) # Blast
                
                # Note: To create a "Pulse", we rely on the loop interaction or actuator blinking.
                # If we want 5Hz, we need config change.
                # For now, let's just trigger a strong alert: HIGH BEAM + HORN.
                
                # We return early or allow overlap?
                # Hazard overrides Overtaking logic.
                
                # Ensure turn signals are not overridden if we are stalling? 
                # If Stalling AND Hazard? 
                # Stall sets Hazard Lights. Moving Hazard sets Beams/Horn.
                # They operate on different Actuators (Models).
                # `TurnSignalActuator` vs `BeamActuator`.
                
                # So we can do both.
                
            else:
                # NO HAZARD -> Normal Logic
                
                # Only run normal logic if NOT Stalled
                if not (engine_rpm < 300 and speed > 1.0):

                    # 2. Update Overtaking Status
                    self.overtaking_detection.process_update(ego_position, tracked_vehicles)
                    overtaking_status = self.overtaking_detection.get_overtaking_status()
                    
                    # 3. Determine Beam Mode (Logic)
                    target_mode = self.decision_engine.determine_beam_mode(tracked_vehicles, overtaking_status)
                    
                    # 4. Handle Blinking and Horn Logic
                    self._handle_overtaking_actions(overtaking_status, tracked_vehicles, target_mode)
                    
                    # 5. Apply Beam Mode (if not blinking)
                    if not self.beam_actuator.is_blinking():
                        self.beam_actuator.switch_beam_mode(target_mode)
                    
                    # 6. Automatic Turn Signals
                    self._handle_turn_signals(lateral_velocity, steering_angle, yaw_rate)
            
        except Exception as e:
            self.error_handler.log_error(f"System Failure: {str(e)}", 500)
            self.state = SystemState.ERROR
            self.beam_actuator.switch_beam_mode(BeamMode.LOW_BEAM)
            self.beam_actuator.stop_blinking()
            self.horn_actuator.stop_horn()
            self.turn_signal_actuator.activate_signal(TurnSignalState.OFF)
            
        # 7. Update Actuators
        self.beam_actuator.update(dt)
        self.horn_actuator.update(dt)
        self.turn_signal_actuator.update(dt)

    def _handle_turn_signals(self, lateral_velocity: float, steering_angle: float, yaw_rate: float):
        """
        Activates turn signals based on:
        1. Lateral Velocity (Lane Change)
        2. Steering Angle (Turning)
        3. Yaw Rate (Turning)
        
        Logic:
        - Activate if ANY metric exceeds threshold.
        - Cancel if ALL metrics are below threshold (Return to center).
        """
        # Thresholds
        lat_vel_thresh = 0.5 # m/s (Lane Change)
        steering_thresh = 15.0 # degrees (Turning)
        yaw_thresh = 5.0 # deg/s (Turning)
        
        # Deadzones for cancellation
        steering_deadzone = 5.0
        yaw_deadzone = 2.0
        lat_vel_deadzone = 0.2

        current_signal = self.turn_signal_actuator.get_state()
        new_signal = current_signal

        # Activation Logic
        is_turning_left = (
            lateral_velocity > lat_vel_thresh or 
            steering_angle > steering_thresh or 
            yaw_rate > yaw_thresh
        )
        
        is_turning_right = (
            lateral_velocity < -lat_vel_thresh or 
            steering_angle < -steering_thresh or 
            yaw_rate < -yaw_thresh
        )

        if is_turning_left:
            new_signal = TurnSignalState.LEFT
        elif is_turning_right:
            new_signal = TurnSignalState.RIGHT
        
        # Cancellation Logic (Auto-Cancel)
        if current_signal in [TurnSignalState.LEFT, TurnSignalState.RIGHT]:
            # Should we cancel? Only if NOT turning anymore.
            if not is_turning_left and not is_turning_right:
                 # Check if we are "Straight"
                 is_straight = (
                     abs(lateral_velocity) < lat_vel_deadzone and
                     abs(steering_angle) < steering_deadzone and
                     abs(yaw_rate) < yaw_deadzone
                 )
                 if is_straight:
                     new_signal = TurnSignalState.OFF

        self.turn_signal_actuator.activate_signal(new_signal)

    def _handle_overtaking_actions(self, status: OvertakingStatus, vehicles: List[DetectedVehicle], target_mode_from_logic: BeamMode):
        """
        Coordinates blinking and horn during overtaking.
        """
        if status == OvertakingStatus.IN_PROGRESS:
            # Check for oncoming vehicles
            has_oncoming = any(v.type == VehicleType.ONCOMING for v in vehicles)
            
            if has_oncoming:
                # Pause blinking, switch to Low Beam
                if self.beam_actuator.is_blinking():
                    self.beam_actuator.stop_blinking()
                # Enforce Low Beam
                self.beam_actuator.switch_beam_mode(BeamMode.LOW_BEAM)
            else:
                target_id = self.overtaking_detection.get_target_vehicle()
                target_vehicle = self.vehicle_detection.get_vehicle_by_id(target_id) if target_id else None
                
                # Start blinking if we are overtaking a valid target
                if target_vehicle and not self.beam_actuator.is_blinking():
                    self.beam_actuator.start_blinking()
                    self.last_blink_cycle = int(self.beam_actuator.blinking_timer / (1.0/self.config.blinking_frequency)) - 1

                # Sync Horn if blinking
                if self.beam_actuator.is_blinking():
                    self._sync_horn()
                    
        else:
            # Not overtaking -> Stop blinking
            if self.beam_actuator.is_blinking():
                self.beam_actuator.stop_blinking()

    def _sync_horn(self):
        """
        Triggers horn synchronized with blinking high beam.
        """
        period = 1.0 / self.config.blinking_frequency
        current_timer = self.beam_actuator.blinking_timer
        
        current_cycle = int(current_timer / period)
        
        if current_cycle > self.last_blink_cycle:
             # New cycle started.
             horn_duration = (self.config.horn_pulse_duration_min + self.config.horn_pulse_duration_max) / 2
             self.horn_actuator.trigger_horn(horn_duration)
             self.last_blink_cycle = current_cycle

    def set_manual_override(self, enabled: bool):
        if enabled:
            self.state = SystemState.MANUAL_OVERRIDE
            self.beam_actuator.stop_blinking()
            self.horn_actuator.stop_horn()
            self.turn_signal_actuator.activate_signal(TurnSignalState.OFF)
        else:
            self.state = SystemState.NORMAL
