# Automatic Headlight Control System

A robust, intelligent headlight control system designed to automate beam switching (High/Low) and signaling (Blinking/Horn) during overtaking maneuvers. This system prioritizes safety by reacting to oncoming traffic and vehicle proximity.

## Features

- **Intelligent Beam Switching**: Automatically toggles between High and Low Beam based on surroundings.
- **Overtaking Assistance**: Detects overtaking maneuvers and activates beam blinking (2Hz) and synchronized horn pulses to alert vehicles ahead.
- **Automatic Turn Signals**: Activates Left/Right turn signals automatically based on lateral vehicle movement during overtaking.
- **Active Hazard Avoidance**:
    -   Detects oncoming vehicles blocking the lane.
    -   Triggers immediate **High Beam Pulse + Horn** to warn the other driver, forcing them to yield.
- **Engine Stall Protection**:
    -   Detects engine stalls (RPM drop while moving).
    -   Automatically activates **Hazard Lights** to warn surrounding traffic.
- **Safety First**:
    -   Switches to Low Beam immediately when oncoming traffic is detected.
    -   Prevents glare for vehicles ahead within unsafe distance.
    -   Fails-safe to Low Beam in case of system error.
- **Robustness**: Validated with property-based testing (`hypothesis`) to ensure reliability across edge cases.

## System Requirements

The system adheres to the following key requirements:

### Functional
- **R1 - Vehicle Detection**: Track position and distance of vehicles ahead and oncoming traffic.
- **R2 - Overtaking Detection**: Identify lane changes to initiate overtaking sequences.
- **R3/R4/R5 - Beam Control**:
    - **High Beam**: Activate during safe overtaking.
    - **Low Beam**: Activate if oncoming traffic is detected OR vehicle ahead is within 50m.
- **R7 - Manual Override**: Driver can force a beam mode, overriding automatic logic.
- **R9 - Signaling**:
    - **Blinking**: Toggle High/Low beam at **2Hz** during overtaking.
    - **Horn**: Trigger a **200-300ms** horn pulse synchronized with each High Beam flash.
- **R11 - Turn Signals**: Automatically activate Left/Right indicators when lateral velocity exceeds **0.5 m/s**.

### Performance & Safety
- **R8 - Response Time**: System processing and actuation signals must occur within **200ms** of environmental changes.
- **R10 - Fail-Safe**:
    - Default to **Low Beam** on system startup or error.
    - If sensors fail, disable automation and maintain Low Beam.
    - Pause blinking immediately if oncoming traffic appears (Req 9.6).

## Project Structure

```
├── src/
│   ├── actuators.py          # Beam and Horn physical state management
│   ├── config.py             # Configuration and sensor interfaces
│   ├── controller.py         # Main system controller (Orchestrator)
│   ├── decision_engine.py    # Logic for safe beam mode determination
│   ├── error_handling.py     # System error logging and state management
│   ├── models.py             # Core data models (DetectedVehicle, Position, etc.)
│   ├── overtaking_detection.py # Logic to detect overtaking maneuvers
│   └── vehicle_detection.py  # Sensor data processing and tracking
├── tests/                    # Unit and Integration tests
├── verify_system.py          # Simulation script for demonstration
└── README.md
```

## Installation

Ensure you have Python 3.8+ installed.

1.  Clone the repository.
2.  Install dependencies (for testing):

```bash
pip install pytest hypothesis
```

## Usage

### Running the Simulation

To see the system in action, run the verification script. This simulates a vehicle overtaking scenario with console output showing the beam state, blinking status, and horn activation.

```bash
python verify_system.py
```

**Output Example:**
```text
T=3.2s | EgoX: 2.0 | RelY: 10.0 | Status: IN_PROGRESS  | Beam: HIGH_BEAM  | BLINK | HONK
**Output Example:**
```text
T=3.2s | EgoX: 2.0 | RelY: 10.0 | Status: IN_PROGRESS  | Beam: HIGH_BEAM  | BLINK | HONK
```

### Visual Demo
To view a visual animation of the system in action:
1.  Open the file `simulation_viz.html` in your web browser.
2.  Watch the simulation of the overtaking maneuver, including beam switching and turn signals.

### Running Tests

The project includes a comprehensive test suite covering unit logic, property-based validation, and full system integration.

```bash
pytest
```

## Configuration

System parameters can be adjusted in `src/models.py` (default values) or loaded via `src/config.py`.

- `detection_range`: 200m
- `safe_distance`: 50m
- `blinking_frequency`: 2Hz
- `horn_pulse_duration`: 0.2-0.3s

## Real World Integration

In a real vehicle deployment, this software manages the logic layer of the headlight system. Here is how it interfaces with physical hardware:

### 1. Sensor Input (The "Eyes")
- **Hardware**: Cameras (Mobileye), Radar (Bosch/Continental), or LiDAR.
- **Data Flow**: These sensors process raw point clouds/images and output an **Object List** (Distance, Velocity, Type) via the **CAN Bus**.
- **Integration**: The `SensorInterface` in `src/config.py` would be replaced by a `CANBusAdapter` that reads these signals and populates `DetectedVehicle` objects.

### 2. Computing Unit (The "Brain")
- **Hardware**: This python code (transpiled to C/C++ or running on embedded Linux) runs on an **ECU** (Electronic Control Unit), such as the **Body Control Module (BCM)** or a dedicated ADAS controller.
- **Timing**: The `process_update` loop runs at **10Hz - 100Hz** (Real-Time) to ensure reaction times under 200ms.

### 3. Actuation (The "Hands")
- **Headlights**:
    - **High/Low Beam**: Signals are sent to the headlight driver. In modern matrix LED systems, this might enable/disable specific pixel blocks to shadow cars instead of just switching low/high.
- **Horn**:
    - A signal is sent to the Horn Relay or BCM to trigger the physical horn sounded for the calculated duration.

### 4. Safety & Redundancy
- **Fail-Safe**: If the camera stops sending data (CAN Timeout), the `ErrorHandler` triggers, and the system defaults to **Low Beam** (Hardware Default) to prevent blinding others.
- **Watchdog**: A hardware watchdog monitors this software. If the code hangs, the watchdog resets the controller, forcing a default Safe State (Low Beam).

## Vehicle Compatibility

This software is designed to be **platform-agnostic**, meaning the core logic works on any vehicle type (Car, Truck, Motorcycle) provided it meets the following hardware requirements:

1.  **Sensors**: Must have at least one forward-facing sensor (Camera, Radar, or LiDAR) capable of detecting:
    -   Object Type (Car, Truck, Motorcycle)
    -   Relative Distance & Velocity
    -   Lateral Position (Lane tracking)
2.  **Digital Control**:
    -   **Headlights**: Must support electronic switching of High/Low beams (Relay or CAN bus controlled).
    -   **Horn**: Must support electronic triggering (Relay-based).
3.  **Connectivity**: An onboard computer or ECU capable of interfacing with the vehicle's CAN bus or other communication networks.

**Note**: The detection ranges (`200m`) and safety distances (`50m`) are configurable in `src/config.py` to adapt to different vehicle dynamics (e.g., a truck may need longer braking distances).
