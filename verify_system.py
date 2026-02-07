import time
import math
from src.models import Configuration, DetectedVehicle, Position, Velocity, BeamMode, OvertakingStatus, VehicleType
from src.controller import HeadlightController
from src.vehicle_detection import VehicleDetectionModule
from src.overtaking_detection import OvertakingDetectionModule
from src.decision_engine import DecisionEngine
from src.actuators import BeamActuatorModule, HornActuatorModule, TurnSignalActuatorModule
from src.error_handling import ErrorHandler

def run_simulation():
    print("Initializing Automatic Headlight Control System...")
    config = Configuration(blinking_frequency=2.0)
    vd = VehicleDetectionModule(config)
    od = OvertakingDetectionModule(config)
    de = DecisionEngine(config)
    ba = BeamActuatorModule(config)
    ha = HornActuatorModule(config)
    tsa = TurnSignalActuatorModule(config)
    eh = ErrorHandler()
    
    controller = HeadlightController(config, vd, od, de, ba, ha, tsa, eh)
    
    print("System Initialized. Default Mode:", controller.beam_actuator.current_mode)
    
    # Simulation Loop
    dt = 0.1
    sim_time = 0.0
    
    # Scenario: Overtaking a vehicle
    print("\n--- Starting Simulation: Overtaking Scenario ---")
    
    # Initial State
    # Ego at 0, 20m/s
    ego_pos = Position(0.0, 0.0)
    ego_vel_y = 20.0
    
    # Target at 0, 50m ahead, moving 15m/s
    target_pos_y = 50.0
    target_vel_y = 15.0
    v1 = DetectedVehicle(1, VehicleType.AHEAD, Position(0, target_pos_y), Velocity(0, target_vel_y), 50.0)
    
    lateral_pos = 0.0
    
    # Scenario 1: Overtaking (0-10s)
    # Scenario 2: Hazard (12-14s)
    # Scenario 3: Engine Stall (16-18s)
    
    for i in range(200): # 20 seconds
        sim_time += dt
        
        # Default Inputs
        steering = 0.0
        yaw = 0.0
        rpm = 2500.0 # Normal RPM
        
        # 1. Update Longitudinal Positions
        ego_pos_y = ego_pos.y + ego_vel_y * dt
        target_pos_y += target_vel_y * dt
        
        # 2. Manage Lateral Position (Scenario Logic)
        if 1.0 <= sim_time < 3.0:
            lateral_pos += (3.5 / 2.0) * dt
        elif 6.0 <= sim_time < 8.0:
            lateral_pos -= (3.5 / 2.0) * dt
            
        ego_pos = Position(lateral_pos, ego_pos_y)
        
        # 3. Update V1 for Detection
        v1_list = []
        
        # SCENARIO 1: Overtaking Target (Always there)
        rel_x = 0.0 - ego_pos.x
        rel_y = target_pos_y - ego_pos.y
        v1_input = DetectedVehicle(1, VehicleType.AHEAD, Position(rel_x, rel_y), Velocity(0, target_vel_y), math.sqrt(rel_x**2 + rel_y**2))
        v1_list.append(v1_input)
        
        # SCENARIO 2: Oncoming Hazard (12-14s)
        if 12.0 <= sim_time < 14.0:
             # Oncoming car in EGO LANE (lateral_pos is ~0)
             # V2 approaching fast
             rel_x_haz = 0.0 - ego_pos.x # In front of us
             rel_y_haz = 100.0 - (sim_time - 12.0) * 40.0 # Approaching from 100m at 40m/s relative
             v_haz = DetectedVehicle(
                 2, VehicleType.ONCOMING, 
                 Position(rel_x_haz, rel_y_haz), 
                 Velocity(0, -20.0), # Absolute velocity -20m/s (Towards us)
                 math.sqrt(rel_x_haz**2 + rel_y_haz**2)
             )
             v1_list.append(v_haz)
             
        # SCENARIO 3: Engine Stall (16s+)
        if sim_time >= 16.0:
            rpm = 0.0 # STALL
            # Ego velocity should drop, but for simple sim we keep it > 1.0 to trigger logic
            
        # Inject Steering/Yaw Data for Demo
        # Turn Left (1-1.5s)
        if 1.0 <= sim_time < 1.5:
            steering = 20.0; yaw = 6.0
        # Turn Right (6-6.5s)
        if 6.0 <= sim_time < 6.5:
            steering = -20.0; yaw = -6.0
            
        # 4. Run Cycle
        # Ego Velocity needed for hazard check
        ego_vel = Velocity(0, ego_vel_y)
        controller.process_update(ego_pos, v1_list, dt, steering_angle=steering, yaw_rate=yaw, ego_velocity=ego_vel, engine_rpm=rpm)
        
        # 5. Print Status
        status = controller.overtaking_detection.get_overtaking_status()
        beam = controller.beam_actuator.get_current_physical_beam()
        blinking = "BLINK" if controller.beam_actuator.is_blinking() else "OFF"
        horn = "HONK" if controller.horn_actuator.is_horn_active() else "---"
        turn_signal = controller.turn_signal_actuator.get_state().name
        
        print(f"T={sim_time:4.1f}s | RPM:{rpm:4.0f} | Status: {status.name:12} | Beam: {beam.name:10} | {blinking} | {horn} | Signal: {turn_signal}")
        
        time.sleep(0.01)

if __name__ == "__main__":
    run_simulation()
