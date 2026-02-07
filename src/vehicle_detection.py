import math
from typing import Dict, List, Optional
from src.models import DetectedVehicle, Position, Velocity, VehicleType, Configuration

class VehicleDetectionModule:
    def __init__(self, config: Configuration):
        self.config = config
        self.tracked_vehicles: Dict[int, DetectedVehicle] = {}

    def process_sensor_data(self, raw_vehicles: List[DetectedVehicle]) -> List[DetectedVehicle]:
        """
        Process a list of potential vehicles from sensors.
        Updates tracked vehicles, adds new ones, and removes those out of range.
        Returns the current list of valid tracked vehicles.
        """
        current_frame_ids = set()

        for vehicle in raw_vehicles:
            if not vehicle.is_valid():
                continue

            distance = self._calculate_distance(vehicle.position)
            
            if self._is_within_range(distance):
                # Update or register vehicle
                # Ensure distance is updated
                vehicle.distance = distance
                self.tracked_vehicles[vehicle.id] = vehicle
                current_frame_ids.add(vehicle.id)
        
        # Remove vehicles that are no longer detected or out of range
        # (Assuming raw_vehicles contains ALL currently visible vehicles)
        # If a vehicle is missing from raw_vehicles, we assume it's gone/out of range per this simple logic
        # OR we could implement a persistence time-to-live, but for now strict visibility:
        
        # However, requirements say: "WHEN a detected vehicle exits the detection range... remove"
        # So we should also check if tracked vehicles are in current frame.
        
        ids_to_remove = set(self.tracked_vehicles.keys()) - current_frame_ids
        for id in ids_to_remove:
            del self.tracked_vehicles[id]

        return list(self.tracked_vehicles.values())

    def _is_within_range(self, distance: float) -> bool:
        return distance <= self.config.detection_range

    def _calculate_distance(self, position: Position) -> float:
        # Assuming ego vehicle is at (0,0) concept
        return math.sqrt(position.x**2 + position.y**2)

    def get_vehicle_by_id(self, id: int) -> Optional[DetectedVehicle]:
        return self.tracked_vehicles.get(id)

    def detect_hazards(self, ego_velocity: Velocity) -> tuple:
        """
        Scans tracked vehicles for potential collision hazards.
        Returns: (HazardType, vehicle_id, description)
        """
        from src.models import HazardType

        for v in self.tracked_vehicles.values():
            # 1. Determine if in Ego Lane (approx 3.5m width -> +/- 1.75m)
            # Use strict 1.5m buffer
            in_ego_lane = abs(v.position.x) < 1.5

            if in_ego_lane:
                # Relative Velocity (Closing Speed) calculation
                # Assuming v.velocity is absolute ground speed
                # Closing Speed = Ego - Target (Positive means closing on it)
                
                # Handling ONCOMING
                if v.type == VehicleType.ONCOMING:
                    # Oncoming in same lane is CRITICAL
                    # Even if far away, it's a hazard if moving towards us
                    # Relative speed = Ego - (-Target) = Ego + Target
                    # If v.velocity.y is negative (coming down) and ego is positive (going up)
                    
                    # Check if actually approaching
                    # If v.y > 0 and v.velocity.y < 0: Approaching
                    is_approaching = (v.position.y > 0 and v.velocity.vy < 0) or (v.position.y < 0 and v.velocity.vy > 0)
                    
                    if is_approaching:
                        return (HazardType.BLOCKING_VEHICLE, v.id, "Oncoming Vehicle in Lane!")

                # Handling AHEAD (Overtaking blocked)
                elif v.type == VehicleType.AHEAD:
                    closing_speed = ego_velocity.vy - v.velocity.vy
                    
                    # CRITICAL if close and closing fast
                    # Dist < 30m and Closing > 5m/s (18km/h diff)
                    if v.distance < 30.0 and closing_speed > 5.0:
                         return (HazardType.BLOCKING_VEHICLE, v.id, f"Rapidly approaching vehicle ahead! Closing speed: {closing_speed:.1f} m/s")
                    
                    # BLOCKING if very close regardless of speed
                    if v.distance < 10.0:
                        return (HazardType.BLOCKING_VEHICLE, v.id, "Vehicle ahead critically close!")

        return (HazardType.NONE, None, "")
