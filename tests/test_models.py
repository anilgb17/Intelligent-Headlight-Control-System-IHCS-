from hypothesis import given, strategies as st
import math
from src.models import Position, Velocity, DetectedVehicle, VehicleType

# Strategies
position_strategy = st.builds(Position, x=st.floats(min_value=-100, max_value=100), y=st.floats(min_value=0, max_value=500))
velocity_strategy = st.builds(Velocity, vx=st.floats(min_value=-50, max_value=50), vy=st.floats(min_value=-100, max_value=100))
vehicle_type_strategy = st.sampled_from(VehicleType)

@given(x=st.floats(allow_nan=False, allow_infinity=False), 
       y=st.floats(allow_nan=False, allow_infinity=False))
def test_position_creation(x, y):
    pos = Position(x=x, y=y)
    assert pos.x == x
    assert pos.y == y
    assert pos.is_valid()

@given(vx=st.floats(allow_nan=False, allow_infinity=False), 
       vy=st.floats(allow_nan=False, allow_infinity=False))
def test_velocity_creation(vx, vy):
    vel = Velocity(vx=vx, vy=vy)
    assert vel.vx == vx
    assert vel.vy == vy
    assert vel.is_valid()

@given(id=st.integers(min_value=0),
       bg_type=vehicle_type_strategy,
       pos=position_strategy,
       vel=velocity_strategy,
       dist=st.floats(min_value=0, max_value=500))
def test_detected_vehicle_creation(id, bg_type, pos, vel, dist):
    vehicle = DetectedVehicle(id=id, type=bg_type, position=pos, velocity=vel, distance=dist)
    assert vehicle.id == id
    assert vehicle.type == bg_type
    assert vehicle.position == pos
    assert vehicle.velocity == vel
    assert vehicle.distance == dist
    assert vehicle.is_valid()

def test_invalid_vehicle():
    pos = Position(0, 0)
    vel = Velocity(0, 0)
    # Invalid ID
    v1 = DetectedVehicle(id=-1, type=VehicleType.AHEAD, position=pos, velocity=vel, distance=10)
    assert not v1.is_valid()
    
    # Invalid distance
    v2 = DetectedVehicle(id=1, type=VehicleType.AHEAD, position=pos, velocity=vel, distance=-10)
    assert not v2.is_valid()
