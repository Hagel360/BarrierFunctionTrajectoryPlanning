"""
state module to handle state transitions for the drone
"""
from enum import Enum

class DroneState(Enum):
    INACTIVE = 0
    FLYING = 1
    LANDING = 2
    HOVERING = 3
    COLLISION_AVOIDANCE = 4
    TAKEOFF = 5

def handle_state_transition(current_state, next_state):
    valid_transitions = {
        DroneState.INACTIVE: [DroneState.FLYING, DroneState.TAKEOFF],
        DroneState.FLYING: [DroneState.HOVERING, DroneState.LANDING, DroneState.COLLISION_AVOIDANCE],
        DroneState.LANDING: [DroneState.INACTIVE],
        DroneState.HOVERING: [DroneState.FLYING, DroneState.LANDING, DroneState.COLLISION_AVOIDANCE],
        DroneState.COLLISION_AVOIDANCE: [DroneState.FLYING, DroneState.HOVERING, DroneState.LANDING],
        DroneState.TAKEOFF: [DroneState.FLYING]
    }

    if next_state in valid_transitions.get(current_state, []):
        print(f"Transitioning from {current_state.name} to {next_state.name}")
        return next_state
    else:
        print(f"Invalid transition from {current_state.name} to {next_state.name}")
        return current_state