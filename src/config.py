import json
from dataclasses import asdict
from typing import List
from abc import ABC, abstractmethod

from src.models import Configuration, DetectedVehicle

class ConfigLoader:
    @staticmethod
    def save_to_file(config: Configuration, filepath: str):
        with open(filepath, 'w') as f:
            json.dump(asdict(config), f, indent=4)

    @staticmethod
    def load_from_file(filepath: str) -> Configuration:
        with open(filepath, 'r') as f:
            data = json.load(f)
        return Configuration(**data)

class SensorInterface(ABC):
    @abstractmethod
    def get_detected_vehicles(self) -> List[DetectedVehicle]:
        pass

class MockSensor(SensorInterface):
    def __init__(self):
        self.vehicles: List[DetectedVehicle] = []

    def set_vehicles(self, vehicles: List[DetectedVehicle]):
        self.vehicles = vehicles

    def get_detected_vehicles(self) -> List[DetectedVehicle]:
        return self.vehicles
