import pytest
from src.models import Configuration, BeamMode
from src.actuators import BeamActuatorModule, HornActuatorModule

def test_beam_actuator_default():
    config = Configuration()
    actuator = BeamActuatorModule(config)
    assert actuator.current_mode == BeamMode.LOW_BEAM
    assert not actuator.is_blinking()

def test_beam_switch():
    config = Configuration()
    actuator = BeamActuatorModule(config)
    actuator.switch_beam_mode(BeamMode.HIGH_BEAM)
    assert actuator.current_mode == BeamMode.HIGH_BEAM
    assert actuator.get_current_physical_beam() == BeamMode.HIGH_BEAM

def test_beam_blinking_logic():
    config = Configuration(blinking_frequency=2.0) # 0.5s period
    actuator = BeamActuatorModule(config)
    
    actuator.start_blinking()
    assert actuator.is_blinking()
    
    # t=0 -> First half -> High Beam
    assert actuator.get_current_physical_beam() == BeamMode.HIGH_BEAM
    
    # t=0.2 -> Still first half (0.2 < 0.25) -> High Beam
    actuator.update(0.2)
    assert actuator.get_current_physical_beam() == BeamMode.HIGH_BEAM
    
    # t=0.3 -> Second half (0.3 in 0.25-0.5) -> Low Beam
    actuator.update(0.1) # total 0.3
    assert actuator.get_current_physical_beam() == BeamMode.LOW_BEAM
    
    # t=0.6 -> Next cycle first half -> High Beam
    actuator.update(0.3) # total 0.6
    assert actuator.get_current_physical_beam() == BeamMode.HIGH_BEAM
    
    actuator.stop_blinking()
    assert not actuator.is_blinking()
    assert actuator.get_current_physical_beam() == actuator.current_mode # Should return base mode (LOW by default)

def test_horn_actuator_default():
    config = Configuration()
    actuator = HornActuatorModule(config)
    assert not actuator.is_horn_active()

def test_horn_trigger_and_duration():
    config = Configuration()
    actuator = HornActuatorModule(config)
    
    actuator.trigger_horn(0.3)
    assert actuator.is_horn_active()
    
    actuator.update(0.2)
    assert actuator.is_horn_active()
    
    actuator.update(0.15) # Total 0.35
    assert not actuator.is_horn_active()

def test_horn_manual_stop():
    config = Configuration()
    actuator = HornActuatorModule(config)
    
    actuator.trigger_horn(1.0)
    actuator.stop_horn()
    assert not actuator.is_horn_active()
