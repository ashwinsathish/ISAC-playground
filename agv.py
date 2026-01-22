"""
AGV (Automated Guided Vehicle) Model
Implements kinematics, motion planning, and state management.
"""

import numpy as np
from typing import Optional, List, Tuple
from enum import Enum
import config as cfg


class AGVState(Enum):
    """AGV operational states."""
    IDLE = 0
    MOVING = 1
    SLOWING = 2
    STOPPED = 3
    EMERGENCY_STOP = 4


class AGV:
    """
    Automated Guided Vehicle with kinematic model.
    Uses bicycle model for motion simulation.
    """
    
    def __init__(self, agv_id: int, initial_position: np.ndarray):
        """
        Initialize an AGV.
        
        Args:
            agv_id: Unique identifier
            initial_position: Initial [x, y] position
        """
        self.id = agv_id
        
        # Kinematic state
        self.position = initial_position.copy().astype(float)
        self.velocity = np.zeros(2)
        self.speed = 0.0
        self.heading = np.random.uniform(0, 2 * np.pi)  # Random initial heading
        
        # Physical properties
        self.length = cfg.AGV_LENGTH
        self.width = cfg.AGV_WIDTH
        self.height = cfg.AGV_HEIGHT
        self.rcs = cfg.AGV_RCS
        
        # Motion constraints
        self.max_velocity = cfg.AGV_MAX_VELOCITY
        self.max_acceleration = cfg.AGV_MAX_ACCELERATION
        self.max_angular_velocity = cfg.AGV_MAX_ANGULAR_VELOCITY
        
        # Navigation
        self.target_waypoint: Optional[np.ndarray] = None
        self.path: List[np.ndarray] = []
        self.state = AGVState.IDLE
        
        # Collision avoidance - SIMPLE lateral offset (Keep Right rule)
        self.speed_limit_factor = 1.0  # 1.0 = normal, <1.0 = slowed
        self.lateral_offset = 0.0  # Positive = offset to the right of heading
        
        # History for visualization
        self.position_history: List[np.ndarray] = []  
        self.max_history = cfg.VIZ_TRAIL_LENGTH
        
    def set_target(self, waypoint: np.ndarray):
        """Set new target waypoint."""
        self.target_waypoint = waypoint.copy()
        self.state = AGVState.MOVING
        
    def set_avoidance_command(self, speed_factor: float, lateral_offset: float):
        """
        Set collision avoidance parameters.
        
        Args:
            speed_factor: Speed multiplier (0 to 1). 0 = STOP
            lateral_offset: Offset perpendicular to heading (positive = right)
        """
        self.speed_limit_factor = np.clip(speed_factor, 0.0, 1.0)  # Allow 0 for stop
        self.lateral_offset = lateral_offset
        
        if speed_factor == 0:
            self.state = AGVState.STOPPED
        elif speed_factor < 1.0:
            self.state = AGVState.SLOWING
        else:
            self.state = AGVState.MOVING
            
    def update(self, dt: float, factory_floor) -> None:
        """
        Update AGV state for one time step.
        
        Args:
            dt: Time step in seconds
            factory_floor: Reference to factory environment
        """
        # Record position history
        self.position_history.append(self.position.copy())
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
        
        # IMMEDIATE STOP if commanded (collision avoidance)
        if self.speed_limit_factor == 0:
            self.speed = max(0, self.speed - self.max_acceleration * 3 * dt)  # Emergency decel
            self.velocity = self.speed * np.array([np.cos(self.heading), np.sin(self.heading)])
            return  # Don't move toward target
            
        # Check if we need a new waypoint
        if self.target_waypoint is None:
            self._get_new_waypoint(factory_floor)
            
        if self.target_waypoint is None:
            self.state = AGVState.IDLE
            return
            
        # Calculate direction to target
        direction = self.target_waypoint - self.position
        distance = np.linalg.norm(direction)
        
        # Check if reached waypoint
        if distance < 0.5:  # Within 50 cm
            self.target_waypoint = None
            self.lateral_offset = 0.0  # Reset offset
            self._get_new_waypoint(factory_floor)
            return
        
        # Calculate EFFECTIVE target with lateral offset (Keep Right rule)
        if abs(self.lateral_offset) > 0.01 and distance > 1.0:
            # Get perpendicular direction (90° clockwise = right)
            forward_dir = direction / distance
            right_dir = np.array([forward_dir[1], -forward_dir[0]])
            
            # Apply offset to create effective target
            effective_target = self.target_waypoint + right_dir * self.lateral_offset
        else:
            effective_target = self.target_waypoint
        
        # Update heading towards EFFECTIVE target
        effective_dir = effective_target - self.position
        target_heading = np.arctan2(effective_dir[1], effective_dir[0])
        heading_diff = self._wrap_angle(target_heading - self.heading)
        
        # Limit angular velocity
        max_turn = self.max_angular_velocity * dt
        heading_change = np.clip(heading_diff, -max_turn, max_turn)
        self.heading = self._wrap_angle(self.heading + heading_change)
        
        # Calculate target speed (slow down near waypoint)
        effective_max_speed = self.max_velocity * self.speed_limit_factor
        stopping_distance = self.speed**2 / (2 * self.max_acceleration)
        
        if distance < stopping_distance + 0.5:
            target_speed = min(effective_max_speed, 
                             np.sqrt(2 * self.max_acceleration * distance))
        else:
            target_speed = effective_max_speed
            
        # Also slow down if not facing target
        if abs(heading_diff) > np.radians(30):
            target_speed *= 0.3
        elif abs(heading_diff) > np.radians(10):
            target_speed *= 0.7
            
        # Accelerate/decelerate towards target speed
        speed_diff = target_speed - self.speed
        max_speed_change = self.max_acceleration * dt
        speed_change = np.clip(speed_diff, -max_speed_change, max_speed_change)
        self.speed = max(0, self.speed + speed_change)
        
        # Update velocity vector
        self.velocity = self.speed * np.array([np.cos(self.heading), 
                                                np.sin(self.heading)])
        
        # Update position
        new_position = self.position + self.velocity * dt
        
        # Check if new position is valid
        if factory_floor.is_position_valid(new_position, margin=self.width/2):
            self.position = new_position
        else:
            # Stop and get new waypoint
            self.speed = 0
            self.velocity = np.zeros(2)
            self.target_waypoint = None
            self.lateral_offset = 0.0
            
        # Reset avoidance for next cycle (will be set again if needed)
        self.speed_limit_factor = 1.0
        self.lateral_offset = 0.0
        if self.speed > 0:
            self.state = AGVState.MOVING
            
    def _decelerate(self, dt: float, emergency: bool = False):
        """Decelerate the AGV."""
        decel = self.max_acceleration * (3.0 if emergency else 1.0)
        self.speed = max(0, self.speed - decel * dt)
        self.velocity = self.speed * np.array([np.cos(self.heading), 
                                                np.sin(self.heading)])
        if self.speed == 0:
            self.state = AGVState.STOPPED
            
    def _get_new_waypoint(self, factory_floor):
        """Get a new random waypoint."""
        self.target_waypoint = factory_floor.get_random_waypoint()
        
    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-π, π]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def get_3d_position(self) -> np.ndarray:
        """Get 3D position (x, y, z) with z at AGV center height."""
        return np.array([self.position[0], self.position[1], self.height / 2])
    
    def get_corners(self) -> np.ndarray:
        """Get the four corners of the AGV footprint."""
        cos_h = np.cos(self.heading)
        sin_h = np.sin(self.heading)
        
        # Half dimensions
        hl = self.length / 2
        hw = self.width / 2
        
        # Corners in local frame
        corners_local = np.array([
            [hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]
        ])
        
        # Rotation matrix
        R = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        
        # Transform to global frame
        corners_global = (R @ corners_local.T).T + self.position
        
        return corners_global
    
    def predict_position(self, dt: float) -> np.ndarray:
        """Predict future position assuming constant velocity."""
        return self.position + self.velocity * dt
    
    def get_state_dict(self) -> dict:
        """Get current state as dictionary."""
        return {
            'id': self.id,
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'speed': self.speed,
            'heading': self.heading,
            'state': self.state.name,
            'target': self.target_waypoint.copy() if self.target_waypoint is not None else None
        }


class AGVFleet:
    """Manages a fleet of AGVs."""
    
    def __init__(self, factory_floor, num_agvs: int = None):
        self.factory_floor = factory_floor
        self.agvs: List[AGV] = []
        
        # Use provided num_agvs or fall back to config
        num_agvs = num_agvs if num_agvs is not None else cfg.NUM_AGVS
        
        # Initialize AGVs with positions from config or generate new ones
        for i in range(num_agvs):
            if i < len(cfg.AGV_INITIAL_POSITIONS):
                pos = cfg.AGV_INITIAL_POSITIONS[i]
            else:
                # Generate random position in factory floor
                pos = np.array([
                    np.random.uniform(5, cfg.FACTORY_LENGTH - 5),
                    np.random.uniform(5, cfg.FACTORY_WIDTH - 5)
                ])
            agv = AGV(agv_id=i, initial_position=pos)
            # Assign initial waypoint so it shows immediately in visualization
            agv.target_waypoint = factory_floor.get_random_waypoint()
            agv.state = AGVState.MOVING
            self.agvs.append(agv)
            
    def update(self, dt: float):
        """Update all AGVs."""
        for agv in self.agvs:
            agv.update(dt, self.factory_floor)
            
    def get_all_positions(self) -> np.ndarray:
        """Get positions of all AGVs as Nx2 array."""
        return np.array([agv.position for agv in self.agvs])
    
    def get_all_velocities(self) -> np.ndarray:
        """Get velocities of all AGVs as Nx2 array."""
        return np.array([agv.velocity for agv in self.agvs])
