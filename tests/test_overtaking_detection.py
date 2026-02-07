import pytest
from src.models import Configuration, DetectedVehicle, Position, Velocity, VehicleType, OvertakingStatus
from src.overtaking_detection import OvertakingDetectionModule

def create_vehicle_ahead(id, dist, x=0, y=None):
    if y is None: y = dist
    return DetectedVehicle(
        id=id,
        type=VehicleType.AHEAD,
        position=Position(x, y),
        velocity=Velocity(0, 0),
        distance=dist
    )

def test_initial_state():
    config = Configuration()
    module = OvertakingDetectionModule(config)
    assert module.get_overtaking_status() == OvertakingStatus.NONE

def test_detect_overtaking_start():
    config = Configuration()
    module = OvertakingDetectionModule(config)
    
    # Vehicle ahead, ego in lane
    v1 = create_vehicle_ahead(1, 50)
    module.process_update(Position(0, 0), [v1])
    assert module.get_overtaking_status() == OvertakingStatus.NONE
    
    # Ego changes lane
    module.process_update(Position(2.0, 0), [v1])
    assert module.get_overtaking_status() == OvertakingStatus.IN_PROGRESS
    assert module.get_target_vehicle() == 1

def test_detect_overtaking_complete():
    config = Configuration()
    module = OvertakingDetectionModule(config)
    
    v1 = create_vehicle_ahead(1, 40)
    # Start overtaking
    module.process_update(Position(2.0, 0), [v1])
    assert module.get_overtaking_status() == OvertakingStatus.IN_PROGRESS
    
    # Move forward, passed vehicle (vehicle relative y becomes negative)
    # Note: process_update expects ego absolute position, but vehicle position is relative?
    # No, models.py doesn't specify if DetectedVehicle position is relative or absolute map coords.
    # But usually sensors give relative. my overtaking logic assumed relative y.
    # Let's assume sensor data is updated to be relative to ego.
    
    # Vehicle passed (y = -10), ego still in passing lane
    v1_passed = create_vehicle_ahead(1, 10, y=-10)
    module.process_update(Position(2.0, 100), [v1_passed]) # Ego moved forward in world, but logic uses relative? 
    # Actually my logic uses `target_vehicle.position.y < 0` which is relative.
    
    assert module.get_overtaking_status() == OvertakingStatus.IN_PROGRESS
    
    # Ego returns to lane
    module.process_update(Position(0, 200), [v1_passed])
    assert module.get_overtaking_status() == OvertakingStatus.COMPLETE

def test_detect_overtaking_abort():
    config = Configuration()
    module = OvertakingDetectionModule(config)
    
    v1 = create_vehicle_ahead(1, 40)
    # Start
    module.process_update(Position(2.0, 0), [v1])
    
    # Return to lane without passing (y > 0)
    module.process_update(Position(0, 10), [v1])
    
    assert module.get_overtaking_status() == OvertakingStatus.ABORTED

def test_target_lost_during_overtaking():
    config = Configuration()
    module = OvertakingDetectionModule(config)
    
    v1 = create_vehicle_ahead(1, 40)
    module.process_update(Position(2.0, 0), [v1])
    
    # Target lost (empty list)
    module.process_update(Position(2.0, 10), [])
    
    assert module.get_overtaking_status() == OvertakingStatus.ABORTED # or potentially should handle gracefully
    assert module.get_target_vehicle() is None
