from typing import Optional, List
from src.models import DetectedVehicle, Position, Velocity, OvertakingStatus, VehicleType, Configuration

class OvertakingDetectionModule:
    def __init__(self, config: Configuration):
        self.config = config
        self.status = OvertakingStatus.NONE
        self.target_vehicle_id: Optional[int] = None
        self.initial_lane_position: float = 0.0 # Assuming start in center of lane 0
        
        # Thresholds
        self.lane_change_threshold = 1.5 # meters
        
    def process_update(self, ego_position: Position, tracked_vehicles: List[DetectedVehicle]):
        """
        Update overtaking status based on ego position and tracked vehicles.
        """
        # Finds the closest vehicle ahead if we are not already overtaking
        vehicle_ahead = self._find_closest_vehicle_ahead(tracked_vehicles)
        
        if self.status == OvertakingStatus.NONE:
            self._check_start_overtaking(ego_position, vehicle_ahead)
        elif self.status == OvertakingStatus.IN_PROGRESS:
            target_vehicle = self._find_vehicle_by_id(tracked_vehicles, self.target_vehicle_id)
            if target_vehicle:
                self._check_overtaking_progress(ego_position, target_vehicle)
            else:
                # Target lost? Maybe abort or complete if passed?
                # For safety, if target is lost, we might assume abort or complete based on position?
                # Or just reset if we can't track it.
                # Requirement: "IF the ego vehicle aborts... recognize as incomplete"
                # If target is gone, we can't really say we overtook it unless we saw it pass.
                # Let's abort for now if target is lost.
                self.status = OvertakingStatus.ABORTED # Or just reset?
                self.target_vehicle_id = None

                
        elif self.status == OvertakingStatus.COMPLETE or self.status == OvertakingStatus.ABORTED:
             # Reset logic if needed, or wait for external reset?
             # For now, let's allow it to reset to NONE if we are back in lane and no vehicle immediately ahead?
             # Or just simple state machine:
             if abs(ego_position.x - self.initial_lane_position) < 0.5:
                 self.status = OvertakingStatus.NONE
                 self.target_vehicle_id = None

    def _find_closest_vehicle_ahead(self, vehicles: List[DetectedVehicle]) -> Optional[DetectedVehicle]:
        closest = None
        min_dist = float('inf')
        for v in vehicles:
            if v.type == VehicleType.AHEAD:
                # Assuming vehicle ahead means positive Y relative to ego?
                # Or just distance and type AHEAD.
                # DetectedVehicle.distance is absolute distance.
                # But we should check if it is actually in front (y > 0 relative to ego if ego is at 0,0 and facing +y)
                # Let's assume DetectedVehicle position is relative to ego.
                if v.position.y > 0 and v.distance < min_dist:
                    min_dist = v.distance
                    closest = v
        return closest

    def _find_vehicle_by_id(self, vehicles: List[DetectedVehicle], id: Optional[int]) -> Optional[DetectedVehicle]:
        if id is None: return None
        for v in vehicles:
            if v.id == id:
                return v
        return None

    def _check_start_overtaking(self, ego_pos: Position, vehicle_ahead: Optional[DetectedVehicle]):
        if vehicle_ahead:
            # Check if we are changing lane (lateral move)
            # And vehicle ahead is close enough?
            if abs(ego_pos.x - self.initial_lane_position) > self.lane_change_threshold:
                 # We are changing lane.
                 self.status = OvertakingStatus.IN_PROGRESS
                 self.target_vehicle_id = vehicle_ahead.id

    def _check_overtaking_progress(self, ego_pos: Position, target_vehicle: DetectedVehicle):
        # Passed if target is behind us (y < 0)
        passed = target_vehicle.position.y < 0
        
        # Returned to lane
        returned_to_lane = abs(ego_pos.x - self.initial_lane_position) < 0.5
        
        if passed and returned_to_lane:
            self.status = OvertakingStatus.COMPLETE
            self.target_vehicle_id = None # Completed
        elif not passed and returned_to_lane:
             # Returned to lane without passing - Abort
             self.status = OvertakingStatus.ABORTED
             self.target_vehicle_id = None
        else:
            # Still potentially overtaking
            pass

    def get_overtaking_status(self) -> OvertakingStatus:
        return self.status

    def get_target_vehicle(self) -> Optional[int]:
        return self.target_vehicle_id
