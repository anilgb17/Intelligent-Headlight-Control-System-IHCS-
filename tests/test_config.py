import pytest
import os
import json
from src.models import Configuration, DetectedVehicle, Position, Velocity, VehicleType
from src.config import ConfigLoader, MockSensor

def test_config_save_load(tmp_path):
    config = Configuration(detection_range=300.5, safe_distance=60.0)
    filepath = tmp_path / "config.json"
    
    ConfigLoader.save_to_file(config, str(filepath))
    
    loaded_config = ConfigLoader.load_from_file(str(filepath))
    
    assert loaded_config.detection_range == 300.5
    assert loaded_config.safe_distance == 60.0
    assert loaded_config.update_frequency == 10.0 # Default

def test_mock_sensor():
    sensor = MockSensor()
    v1 = DetectedVehicle(1, VehicleType.AHEAD, Position(0,50), Velocity(0,0), 50)
    
    sensor.set_vehicles([v1])
    vehicles = sensor.get_detected_vehicles()
    
    assert len(vehicles) == 1
    assert vehicles[0].id == 1
