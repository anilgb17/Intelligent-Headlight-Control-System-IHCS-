import pytest
from src.models import Configuration, DetectedVehicle, Position, Velocity, BeamMode, OvertakingStatus, VehicleType, SystemState
from src.controller import HeadlightController
from src.vehicle_detection import VehicleDetectionModule
from src.overtaking_detection import OvertakingDetectionModule
from src.decision_engine import DecisionEngine
from src.actuators import BeamActuatorModule, HornActuatorModule, TurnSignalActuatorModule
from src.error_handling import ErrorHandler

@pytest.fixture
def system():
    config = Configuration(blinking_frequency=2.0, safe_distance=50.0)
    vd = VehicleDetectionModule(config)
    od = OvertakingDetectionModule(config)
    de = DecisionEngine(config)
    ba = BeamActuatorModule(config)
    ha = HornActuatorModule(config)
    tsa = TurnSignalActuatorModule(config)
    eh = ErrorHandler()
    return HeadlightController(config, vd, od, de, ba, ha, tsa, eh)

def create_vehicle(id, type, x, y, dist):
    return DetectedVehicle(
        id=id,
        type=type,
        position=Position(x, y),
        velocity=Velocity(0,0),
        distance=dist
    )

def test_full_overtaking_sequence(system):
    # 1. Follow vehicle (Safe distance, High Beam)
    v1 = create_vehicle(1, VehicleType.AHEAD, 0, 80, 80)
    system.process_update(Position(0,0), [v1], 0.1)
    assert system.beam_actuator.current_mode == BeamMode.HIGH_BEAM
    assert not system.beam_actuator.is_blinking()
    
    # 2. Get closer (Unsafe distance, Low Beam)
    v1 = create_vehicle(1, VehicleType.AHEAD, 0, 40, 40)
    system.process_update(Position(0,40), [v1], 0.1) 
    # Logic: Vehicle Ahead < 50m -> Low Beam
    assert system.beam_actuator.current_mode == BeamMode.LOW_BEAM
    
    # 3. Start Overtaking (Change Lane)
    # Lateral change > 1.5m
    system.process_update(Position(2.0, 40), [v1], 0.1)
    
    # Should start blinking + horn
    assert system.overtaking_detection.get_overtaking_status() == OvertakingStatus.IN_PROGRESS
    assert system.beam_actuator.is_blinking()
    assert system.horn_actuator.is_horn_active() # First cycle horn
    
    # 4. Continue Overtaking (Horn should stop after duration)
    # 0.3s passed total blink time
    system.process_update(Position(2.0, 45), [v1], 0.3) 
    assert system.beam_actuator.is_blinking()
    assert not system.horn_actuator.is_horn_active() # Horn dur ~0.25s
    
    # 5. Pass Vehicle (y relative becomes negative)
    # Vehicle at 40 (relative Y). Ego moves past it.
    # We update vehicle position to be behind ego?
    # Or just ego moves way forward.
    # Ego at Y=100. Vehicle at Y=40 (absolute). Relative = -60.
    v1_passed = create_vehicle(1, VehicleType.AHEAD, 0, -60, 60) # Position is relative in my interpretation for detection?
    # Wait, in `OvertakingDetectionModule._check_overtaking_progress`: `passed = target_vehicle.position.y < 0`
    # And I assumed `process_update` inputs absolute ego position.
    # But `DetectedVehicle` typically comes from sensors relative to ego.
    # My test setup uses relative positions for `create_vehicle` mostly (y=dist).
    # So `v1_passed` with y=-60 means it's 60m behind.
    
    system.process_update(Position(2.0, 100), [v1_passed], 0.1)
    assert system.overtaking_detection.get_overtaking_status() == OvertakingStatus.IN_PROGRESS # Not returned to lane yet
    
    # 6. Return to Lane
    system.process_update(Position(0, 150), [v1_passed], 0.1) # Back to lane 0
    assert system.overtaking_detection.get_overtaking_status() == OvertakingStatus.COMPLETE
    
    # Should stop blinking and be High Beam (if safe)
    assert not system.beam_actuator.is_blinking()
    assert system.beam_actuator.current_mode == BeamMode.HIGH_BEAM

def test_oncoming_interruption(system):
    # Start overtaking sequence
    v1 = create_vehicle(1, VehicleType.AHEAD, 0, 40, 40)
    system.process_update(Position(2.0, 40), [v1], 0.1)
    assert system.beam_actuator.is_blinking()
    
    # Oncoming appears!
    v_oncoming = create_vehicle(2, VehicleType.ONCOMING, -2, 100, 100)
    system.process_update(Position(2.0, 50), [v1, v_oncoming], 0.1)
    
    # Must stop blinking and Low Beam
    assert not system.beam_actuator.is_blinking()
    assert system.beam_actuator.current_mode == BeamMode.LOW_BEAM
    
    # Oncoming passes (gone)
    system.process_update(Position(2.0, 60), [v1], 0.1)
    
    # Resume blinking? 
    # My implementation checks: "if target_vehicle and not is_blinking: start_blinking"
    # So it should auto-resume blinking if still overtaking!
    assert system.beam_actuator.is_blinking()
