import pytest
from src.models import Configuration, DetectedVehicle, Position, Velocity, BeamMode, SystemState
from src.controller import HeadlightController
from src.vehicle_detection import VehicleDetectionModule
from src.overtaking_detection import OvertakingDetectionModule
from src.decision_engine import DecisionEngine
from src.actuators import BeamActuatorModule, HornActuatorModule, TurnSignalActuatorModule
from src.error_handling import ErrorHandler

def test_error_handling_exception():
    config = Configuration()
    vd = VehicleDetectionModule(config)
    od = OvertakingDetectionModule(config)
    de = DecisionEngine(config)
    ba = BeamActuatorModule(config)
    ha = HornActuatorModule(config)
    tsa = TurnSignalActuatorModule(config)
    eh = ErrorHandler()
    
    controller = HeadlightController(config, vd, od, de, ba, ha, tsa, eh)
    
    # Simulate exception in detection (by passing invalid type that causes error internally or mocking)
    # Easiest way: Mock vehicle_detection.process_sensor_data to raise Exception
    # Or just pass something that breaks it if possible.
    # Given strong typing, it's hard to break unless we mock.
    # We can perform a monkeypatch.
    
    def raise_error(*args):
        raise ValueError("Simulated Sensor Failure")
        
    original_method = vd.process_sensor_data
    vd.process_sensor_data = raise_error
    
    # Process update
    controller.process_update(Position(0,0), [], 0.1)
    
    # Should be in Error State
    assert controller.state == SystemState.ERROR
    # Should have logged error
    assert len(eh.get_logs()) == 1
    assert "Simulated Sensor Failure" in eh.get_logs()[0]["message"]
    # Should be in Low Beam
    assert controller.beam_actuator.current_mode == BeamMode.LOW_BEAM
    
    # Restore
    vd.process_sensor_data = original_method

def test_manual_override_error_logging():
    config = Configuration()
    vd = VehicleDetectionModule(config)
    od = OvertakingDetectionModule(config)
    de = DecisionEngine(config)
    ba = BeamActuatorModule(config)
    ha = HornActuatorModule(config)
    tsa = TurnSignalActuatorModule(config)
    eh = ErrorHandler()
    
    controller = HeadlightController(config, vd, od, de, ba, ha, tsa, eh)
    controller.set_manual_override(True)
    
    def raise_error(*args):
        raise ValueError("Manual Mode Failure")
    
    vd.process_sensor_data = raise_error
    
    controller.process_update(Position(0,0), [], 0.1)
    
    # Should log error but stay in manual?
    assert len(eh.get_logs()) == 1
    assert "Manual Mode Failure" in eh.get_logs()[0]["message"]
    assert controller.state == SystemState.MANUAL_OVERRIDE
