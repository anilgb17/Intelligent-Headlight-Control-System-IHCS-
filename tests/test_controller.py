import pytest
from src.models import Configuration, DetectedVehicle, Position, Velocity, BeamMode, OvertakingStatus, VehicleType, SystemState, TurnSignalState
from src.controller import HeadlightController
from src.vehicle_detection import VehicleDetectionModule
from src.overtaking_detection import OvertakingDetectionModule
from src.decision_engine import DecisionEngine
from src.actuators import BeamActuatorModule, HornActuatorModule, TurnSignalActuatorModule

@pytest.fixture
def controller():
    config = Configuration(blinking_frequency=2.0)
    vd = VehicleDetectionModule(config)
    od = OvertakingDetectionModule(config)
    de = DecisionEngine(config)
    ba = BeamActuatorModule(config)
    ha = HornActuatorModule(config)
    tsa = TurnSignalActuatorModule(config)
    return HeadlightController(config, vd, od, de, ba, ha, tsa)

# ... (Existing tests: create_vehicle_ahead, test_initialization, test_blinking_on_overtaking, test_blinking_pause_with_oncoming, test_manual_override) ...
# I need to match the existing content to replace it or append. 
# The simplest is to replace the imports and fixture, and append new test.

def create_vehicle_ahead(id, dist, x=0, y=None):
    if y is None: y = dist
    return DetectedVehicle(
        id=id,
        type=VehicleType.AHEAD,
        position=Position(x, y),
        velocity=Velocity(0, 0),
        distance=dist
    )

def test_initialization(controller):
    assert controller.state == SystemState.NORMAL
    assert controller.beam_actuator.current_mode == BeamMode.LOW_BEAM
    assert controller.turn_signal_actuator.get_state() == TurnSignalState.OFF

def test_blinking_on_overtaking(controller):
    # Setup vehicle ahead
    v1 = create_vehicle_ahead(1, 50)
    
    # 1. Detect vehicle
    controller.process_update(Position(0,0), [v1], 0.1)
    assert not controller.beam_actuator.is_blinking()
    
    # 2. Start overtaking (move lateral)
    # Velocity calc needs multiple updates.
    # To get > 0.5m/s lateral velocity
    # pos 0 -> 2.0 in 0.1s => 20m/s lateral velocity!
    controller.process_update(Position(2.0, 0), [v1], 0.1)
    
    # Overtaking detected -> Blinking started?
    assert controller.overtaking_detection.get_overtaking_status() == OvertakingStatus.IN_PROGRESS
    assert controller.beam_actuator.is_blinking()
    
    # Turn Signal should be LEFT (vx > 0.5)
    assert controller.turn_signal_actuator.get_state() == TurnSignalState.LEFT

    # 3. Check Horn Sync
    assert controller.horn_actuator.is_horn_active()

def test_blinking_pause_with_oncoming(controller):
    v1 = create_vehicle_ahead(1, 40)
    controller.process_update(Position(2.0, 0), [v1], 0.1) # Updates prev_pos to 2.0
    assert controller.beam_actuator.is_blinking()
    
    # Oncoming vehicle appears
    v_oncoming = DetectedVehicle(2, VehicleType.ONCOMING, Position(-2, 100), Velocity(0,0), 100)
    
    controller.process_update(Position(2.0, 10), [v1, v_oncoming], 0.1) # No lateral change
    
    # Should stop blinking and go to Low Beam
    assert not controller.beam_actuator.is_blinking()
    assert controller.beam_actuator.current_mode == BeamMode.LOW_BEAM
    # Turn signal should be OFF (no lateral move)
    assert controller.turn_signal_actuator.get_state() == TurnSignalState.OFF

def test_manual_override(controller):
    controller.set_manual_override(True)
    assert controller.state == SystemState.MANUAL_OVERRIDE
    
    v1 = create_vehicle_ahead(1, 40)
    controller.process_update(Position(2.0, 0), [v1], 0.1)
    
    assert not controller.beam_actuator.is_blinking()
    assert controller.turn_signal_actuator.get_state() == TurnSignalState.OFF

def test_turn_signals_only(controller):
    # Move Right (negative X)
    controller.process_update(Position(0,0), [], 0.1) # Init
    controller.process_update(Position(-1.0, 0), [], 0.1) # -10m/s
    
    assert controller.turn_signal_actuator.get_state() == TurnSignalState.RIGHT
    
    # Stop moving lateral
    controller.process_update(Position(-1.0, 10), [], 0.1)
    assert controller.turn_signal_actuator.get_state() == TurnSignalState.OFF
