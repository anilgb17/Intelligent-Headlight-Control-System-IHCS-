import pytest
from src.models import Configuration, DetectedVehicle, Position, Velocity, BeamMode, OvertakingStatus, VehicleType
from src.decision_engine import DecisionEngine

def create_vehicle(id, type, dist):
    return DetectedVehicle(
        id=id,
        type=type,
        position=Position(0, dist),
        velocity=Velocity(0,0),
        distance=dist
    )

def test_default_safe_conditions():
    config = Configuration()
    engine = DecisionEngine(config)
    # No vehicles
    mode = engine.determine_beam_mode([], OvertakingStatus.NONE)
    assert mode == BeamMode.HIGH_BEAM

def test_oncoming_vehicle_forces_low_beam():
    config = Configuration(detection_range=200)
    engine = DecisionEngine(config)
    
    # Oncoming in range
    v1 = create_vehicle(1, VehicleType.ONCOMING, 150)
    mode = engine.determine_beam_mode([v1], OvertakingStatus.IN_PROGRESS)
    assert mode == BeamMode.LOW_BEAM

def test_vehicle_ahead_unsafe_distance_forces_low_beam():
    config = Configuration(safe_distance=50)
    engine = DecisionEngine(config)
    
    # Vehicle ahead at 40m (unsafe)
    v1 = create_vehicle(1, VehicleType.AHEAD, 40)
    mode = engine.determine_beam_mode([v1], OvertakingStatus.IN_PROGRESS)
    assert mode == BeamMode.LOW_BEAM

def test_vehicle_ahead_safe_distance_allows_high_beam():
    config = Configuration(safe_distance=50)
    engine = DecisionEngine(config)
    
    # Vehicle ahead at 60m (safe)
    v1 = create_vehicle(1, VehicleType.AHEAD, 60)
    mode = engine.determine_beam_mode([v1], OvertakingStatus.IN_PROGRESS)
    assert mode == BeamMode.HIGH_BEAM

def test_multiple_vehicles_priority():
    config = Configuration(safe_distance=50)
    engine = DecisionEngine(config)
    
    # One safe vehicle ahead, one oncoming
    v1 = create_vehicle(1, VehicleType.AHEAD, 60)
    v2 = create_vehicle(2, VehicleType.ONCOMING, 150)
    
    mode = engine.determine_beam_mode([v1, v2], OvertakingStatus.IN_PROGRESS)
    assert mode == BeamMode.LOW_BEAM
