from hypothesis import given, strategies as st
import pytest
from src.models import Configuration, DetectedVehicle, Position, Velocity, VehicleType
from src.vehicle_detection import VehicleDetectionModule

# Strategies
position_strategy = st.builds(Position, x=st.floats(min_value=-100, max_value=100), y=st.floats(min_value=0, max_value=500))
velocity_strategy = st.builds(Velocity, vx=st.floats(min_value=-50, max_value=50), vy=st.floats(min_value=-100, max_value=100))
vehicle_type_strategy = st.sampled_from(VehicleType)

def create_vehicle(id, dist, pos=None):
    if pos is None:
        pos = Position(0, dist)
    return DetectedVehicle(
        id=id,
        type=VehicleType.AHEAD,
        position=pos,
        velocity=Velocity(0,0),
        distance=dist
    )

def test_initialization():
    config = Configuration()
    module = VehicleDetectionModule(config)
    assert module.tracked_vehicles == {}

@given(dist=st.floats(min_value=0, max_value=199.9))
def test_vehicle_registration_within_range(dist):
    config = Configuration(detection_range=200.0)
    module = VehicleDetectionModule(config)
    
    vehicle = create_vehicle(id=1, dist=dist)
    
    processed = module.process_sensor_data([vehicle])
    
    assert len(processed) == 1
    assert processed[0].id == 1
    assert 1 in module.tracked_vehicles

@given(dist=st.floats(min_value=200.1, max_value=1000))
def test_vehicle_ignored_outside_range(dist):
    config = Configuration(detection_range=200.0)
    module = VehicleDetectionModule(config)
    
    vehicle = create_vehicle(id=1, dist=dist)
    
    processed = module.process_sensor_data([vehicle])
    
    assert len(processed) == 0
    assert module.tracked_vehicles == {}

def test_vehicle_deregistration():
    config = Configuration(detection_range=200.0)
    module = VehicleDetectionModule(config)
    
    # Register
    v1 = create_vehicle(id=1, dist=50)
    module.process_sensor_data([v1])
    assert 1 in module.tracked_vehicles
    
    # Move out of range
    v1_far = create_vehicle(id=1, dist=250)
    
    # If the sensor still reports it but it's far:
    processed = module.process_sensor_data([v1_far])
    assert len(processed) == 0
    assert 1 not in module.tracked_vehicles

def test_independent_tracking():
    config = Configuration(detection_range=200.0)
    module = VehicleDetectionModule(config)
    
    v1 = create_vehicle(id=1, dist=50)
    v2 = create_vehicle(id=2, dist=60)
    
    module.process_sensor_data([v1, v2])
    assert len(module.tracked_vehicles) == 2
    
    # v1 disappears (not in list)
    module.process_sensor_data([v2])
    assert len(module.tracked_vehicles) == 1
    assert 2 in module.tracked_vehicles
    assert 1 not in module.tracked_vehicles

def test_distance_calculation():
    config = Configuration()
    module = VehicleDetectionModule(config)
    pos = Position(3, 4)
    dist = module._calculate_distance(pos)
    assert dist == 5.0

def test_invalid_input_vehicle():
    config = Configuration()
    module = VehicleDetectionModule(config)
    # Invalid vehicle (negative ID)
    v_invalid = DetectedVehicle(-1, VehicleType.AHEAD, Position(0,10), Velocity(0,0), 10)
    
    processed = module.process_sensor_data([v_invalid])
    assert len(processed) == 0
