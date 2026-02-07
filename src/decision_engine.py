from typing import List, Optional
from src.models import DetectedVehicle, BeamMode, OvertakingStatus, VehicleType, Configuration

class DecisionEngine:
    def __init__(self, config: Configuration):
        self.config = config

    def determine_beam_mode(self, tracked_vehicles: List[DetectedVehicle], overtaking_status: OvertakingStatus) -> BeamMode:
        """
        Determine the appropriate beam mode based on current environment.
        """
        # Priority: Low Beam if any unsafe condition exists.
        
        if self._should_switch_to_low_beam(tracked_vehicles):
            return BeamMode.LOW_BEAM
        
        # If safe, High Beam
        # We can enable High Beam if we are overtaking OR just generally if safe?
        # Requirements specifically mention High Beam during overtaking.
        # But Req 4.4 mentions "switch back to high beam... if overtaking is still in progress".
        # Let's assume High Beam is allowed whenever safe.
        
        return BeamMode.HIGH_BEAM

    def _should_switch_to_low_beam(self, vehicles: List[DetectedVehicle]) -> bool:
        """
        Check for conditions requiring Low Beam.
        """
        for v in vehicles:
            if v.type == VehicleType.ONCOMING:
                # Any oncoming vehicle in detection range
                # Req 4.1: "WHEN an oncoming vehicle is detected within the detection range... switch to low beam"
                # Assuming tracked_vehicles are already filtered by range by DetectionModule
                return True
            
            if v.type == VehicleType.AHEAD:
                # Vehicle ahead within safe distance
                # Req 5.1: "WHEN the distance to the vehicle ahead is less than safe distance... switch to low beam"
                if v.distance < self.config.safe_distance:
                    return True
        
        return False
