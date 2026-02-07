# Project Details: Intelligent Headlight Control System

This document provides an in-depth technical overview of the Intelligent Headlight Control System, detailing its architecture, data flow, and component responsibilities.

## 1. System Architecture

The system follows a modular architecture centered around a main controller (Orchestrator pattern) that processes sensor inputs and commands actuators based on a decision logic pipeline.

### High-Level Component Diagram

```mermaid
graph TD
    S[Sensors (Camera/Radar)] -->|Raw Data| VD[Vehicle Detection]
    VD -->|Tracked Vehicles| OD[Overtaking Detection]
    VD -->|Tracked Vehicles| DE[Decision Engine]
    
    subgraph Controller [Headlight Controller]
        direction TB
        VD
        OD
        DE
        Logic[Control Logic & State Machine]
        EH[Error Handling]
    end

    OD -->|Status| Logic
    DE -->|Target Beam Mode| Logic
    
    Logic -->|Commands| BA[Beam Actuator]
    Logic -->|Commands| HA[Horn Actuator]
    Logic -->|Commands| TSA[Turn Signal Actuator]
    
    BA --> Headlights
    HA --> Horn
    TSA --> Indicators
```

## 2. Core Components

### 2.1 Headlight Controller (`src/controller.py`)
The central brain of the system. It runs the main `process_update` loop at high frequency (10-100Hz).
- **Responsibilities**:
    - Aggregates sensor data.
    - Manages system state (`NORMAL`, `ERROR`, `MANUAL_OVERRIDE`).
    - Executes safety checks (Stall Protection, Hazard Avoidance).
    - Coordinates the Overtaking Sequence (Blinking + Horn).
    - Dispatches commands to actuators.

### 2.2 Vehicle Detection (`src/vehicle_detection.py`)
Processes raw sensor data into structured `DetectedVehicle` objects.
- **Functions**:
    - **Tracking**: Maintains persistent IDs for vehicles across frames.
    - **Hazard Detection**: Identifies immediate collision risks (e.g., blocking vehicles).
    - **Filtering**: Removes noise and invalid data points.

### 2.3 Overtaking Detection (`src/overtaking_detection.py`)
Analyzes the ego vehicle's position relative to others to identify overtaking maneuvers.
- **States**:
    - `NONE`: Normal driving.
    - `IN_PROGRESS`: Overtaking maneuver detected (approaching or alongside).
    - `COMPLETE`: Maneuver finished.
    - `ABORTED`: Maneuver cancelled.

### 2.4 Decision Engine (`src/decision_engine.py`)
Pure logic component that determines the optimal beam mode.
- **Logic Rule**:
    - **LOW_BEAM**: If *any* vehicle (Oncoming or Ahead) is within `safe_distance` (50m) OR if Oncoming traffic is detected within `detection_range`.
    - **HIGH_BEAM**: Otherwise (Clear road).

### 2.5 Actuators (`src/actuators.py`)
Software abstractions for physical hardware.
- **BeamActuator**: Controls High/Low beam state and manages blinking timers.
- **HornActuator**: Triggers horn pulses.
- **TurnSignalActuator**: Manages Left/Right/Hazard indicators with auto-cancellation logic.

## 3. Data Models (`src/models.py`)

### DetectedVehicle
Represents a tracked object in the environment.
- `id`: Unique identifier.
- `type`: `VehicleType.AHEAD` or `VehicleType.ONCOMING`.
- `position`: `Position(x, y)` relative to ego vehicle.
- `velocity`: `Velocity(vx, vy)` relative speed.
- `distance`: Euclidean distance to object.

### System Configuration (`Configuration`)
Centralized settings for tuning system behavior.
- `detection_range`: Max distance to consider vehicles (default: 200m).
- `safe_distance`: Buffer distance for Low Beam enforcement (default: 50m).
- `blinking_frequency`: Speed of High/Low toggling during overtaking (default: 2Hz).
- `horn_pulse_duration`: Length of horn sound (0.2-0.3s).

## 4. Control Logic & Safety Features

### 4.1 Overtaking Sequence
When `OvertakingStatus` transitions to `IN_PROGRESS`:
1.  **Check Safety**: Ensure no Oncoming traffic is present.
2.  **Activate Blinking**: Toggle High/Low beam at 2Hz.
3.  **Sync Horn**: Trigger a short horn pulse at the start of each High Beam flash.
4.  **Completion**: Return to normal beam logic when status is `COMPLETE`.

### 4.2 Stall Protection
Monitor Engine RPM and Vehicle Speed.
- **Condition**: `RPM < 300` AND `Speed > 1 m/s` (Moving but engine off).
- **Action**: Immediately activate **Hazard Lights** to warn rear traffic.

### 4.3 Hazard Avoidance
Detects `HazardType.BLOCKING_VEHICLE`.
- **Condition**: Vehicle in direct path with high collision risk.
- **Action**:
    - **High Beam Pulse**: Force High Beam on.
    - **Horn Blast**: Trigger continuous or long-duration horn.
    - **Override**: Surpasses all other logic (Overtaking, Standard Beam) for immediate safety.

### 4.4 Automatic Turn Signals
Based on lateral dynamics (`Lateral Velocity`, `Steering Angle`, `Yaw Rate`).
- **Activation**: Triggers Left or Right signal when thresholds are exceeded (e.g., changing lanes).
- **Cancellation**: detailed "Deadzone" logic ensures signals turn off only when the vehicle stabilizes straight.

## 5. Error Handling (`src/error_handling.py`)
A robust error management system ensures fail-safety.
- **Strategy**:
    - Log all errors with severity levels.
    - If a critical error occurs, transition System State to `ERROR`.
    - **Default Safe State**: Force **Low Beam** and disable automation.
