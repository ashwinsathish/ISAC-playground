"""
Decision Making System - Stop-Then-Avoid Collision Avoidance

Behavior:
1. AGVs detect collision risk → STOP (show the conflict)
2. After 2 seconds of deadlock → activate avoidance
3. One AGV moves sideways to let the other pass
4. Both continue to their waypoints
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from enum import Enum
from dataclasses import dataclass
import config as cfg


class SafetyZone(Enum):
    """Safety zone classification."""
    GREEN = 0   # Normal operation
    YELLOW = 1  # Caution - slow down
    RED = 2     # Danger - stop


@dataclass
class CollisionRisk:
    """Represents a potential collision between two AGVs."""
    agv1_id: int
    agv2_id: int
    distance: float
    severity: SafetyZone


class StopThenAvoidSystem:
    """
    Collision avoidance that:
    1. Stops AGVs when they get too close
    2. After deadlock timeout, one AGV moves sideways
    3. They pass each other and continue
    """
    
    def __init__(self):
        # Safety thresholds
        self.red_distance = cfg.SAFETY_ZONE_RED      # 1.5m - STOP
        self.yellow_distance = cfg.SAFETY_ZONE_YELLOW  # 3.0m - slow down
        
        # Deadlock detection
        self.deadlock_timeout = 2.0  # Seconds before avoidance activates
        self.deadlock_timers: Dict[Tuple[int, int], float] = {}  # pair -> time stuck
        
        # Active avoidance commands
        self.avoidance_active: Dict[int, np.ndarray] = {}  # agv_id -> offset target
        
        # Track risks for metrics
        self.active_risks: List[CollisionRisk] = []
        
    def update(self, agv_fleet, dt: float) -> Dict[int, Tuple[float, float]]:
        """
        Update collision avoidance state and compute commands.
        
        Returns:
            Dict mapping AGV ID to (speed_factor, lateral_offset)
        """
        self.active_risks = []
        commands = {agv.id: (1.0, 0.0) for agv in agv_fleet.agvs}
        
        # Track which pairs are currently in conflict
        current_conflicts = set()
        
        # Check all AGV pairs
        for i, agv1 in enumerate(agv_fleet.agvs):
            for agv2 in agv_fleet.agvs[i+1:]:
                distance = np.linalg.norm(agv1.position - agv2.position)
                pair = (min(agv1.id, agv2.id), max(agv1.id, agv2.id))
                
                # Classify risk
                if distance < self.red_distance:
                    severity = SafetyZone.RED
                elif distance < self.yellow_distance:
                    severity = SafetyZone.YELLOW
                else:
                    severity = SafetyZone.GREEN
                
                if severity != SafetyZone.GREEN:
                    self.active_risks.append(CollisionRisk(
                        agv1_id=agv1.id, agv2_id=agv2.id,
                        distance=distance, severity=severity
                    ))
                
                # Handle based on severity
                if severity == SafetyZone.RED:
                    current_conflicts.add(pair)
                    
                    # Check if avoidance already active for either AGV
                    if agv1.id in self.avoidance_active or agv2.id in self.avoidance_active:
                        # Continue avoidance - let them move
                        for agv_id in [agv1.id, agv2.id]:
                            if agv_id in self.avoidance_active:
                                commands[agv_id] = (0.5, 2.0)  # Slow + offset right
                            else:
                                commands[agv_id] = (0.3, 0.0)  # Other waits
                    else:
                        # No avoidance active - STOP both and track deadlock time
                        commands[agv1.id] = (0.0, 0.0)  # STOP
                        commands[agv2.id] = (0.0, 0.0)  # STOP
                        
                        # Update deadlock timer
                        if pair not in self.deadlock_timers:
                            self.deadlock_timers[pair] = 0.0
                            print(f"[RED ZONE] AGV {agv1.id} & AGV {agv2.id} - distance: {distance:.2f}m - STOPPING")
                        self.deadlock_timers[pair] += dt
                        
                        # Show timer progress every 0.5 seconds
                        if int(self.deadlock_timers[pair] * 2) > int((self.deadlock_timers[pair] - dt) * 2):
                            print(f"   [TIMER] Deadlock timer: {self.deadlock_timers[pair]:.1f}s / {self.deadlock_timeout}s")
                        
                        # Check if deadlock timeout reached
                        if self.deadlock_timers[pair] >= self.deadlock_timeout:
                            # Activate avoidance for lower-ID AGV
                            mover_id = min(agv1.id, agv2.id)
                            self.avoidance_active[mover_id] = self._compute_escape_direction(
                                agv_fleet.agvs[mover_id],
                                agv_fleet.agvs[max(agv1.id, agv2.id)]
                            )
                            # Allow mover to move
                            commands[mover_id] = (0.5, 2.0)
                            print(f"[AVOIDANCE] AGV {mover_id} moving sideways!")
                            
                elif severity == SafetyZone.YELLOW:
                    # Slow down
                    for agv_id in [agv1.id, agv2.id]:
                        current = commands[agv_id]
                        commands[agv_id] = (min(current[0], 0.5), current[1])
        
        # Clear deadlock timers for pairs no longer in conflict
        for pair in list(self.deadlock_timers.keys()):
            if pair not in current_conflicts:
                del self.deadlock_timers[pair]
        
        # Clear avoidance when AGVs are far enough apart
        for agv_id in list(self.avoidance_active.keys()):
            agv = agv_fleet.agvs[agv_id]
            still_close = False
            for other in agv_fleet.agvs:
                if other.id != agv_id:
                    if np.linalg.norm(agv.position - other.position) < self.yellow_distance:
                        still_close = True
                        break
            if not still_close:
                del self.avoidance_active[agv_id]
        
        return commands
    
    def _compute_escape_direction(self, mover, other) -> np.ndarray:
        """Compute which direction the mover should go to escape."""
        # Go perpendicular to the line between them, to the right of mover's heading
        to_other = other.position - mover.position
        # Right of mover's heading
        right = np.array([np.cos(mover.heading - np.pi/2), 
                         np.sin(mover.heading - np.pi/2)])
        return right
    
    def get_situational_awareness(self) -> Dict:
        """Get current situational awareness summary."""
        return {
            'total_risks': len(self.active_risks),
            'red_zone_count': sum(1 for r in self.active_risks if r.severity == SafetyZone.RED),
            'yellow_zone_count': sum(1 for r in self.active_risks if r.severity == SafetyZone.YELLOW),
            'min_distance': min((r.distance for r in self.active_risks), default=float('inf')),
            'risk_pairs': [(r.agv1_id, r.agv2_id, r.severity.name) for r in self.active_risks],
            'deadlock_timers': dict(self.deadlock_timers),
            'avoidance_active': list(self.avoidance_active.keys())
        }


class DecisionMaker:
    """High-level decision making system."""
    
    def __init__(self, localization_system, factory_floor=None):
        self.localization = localization_system
        self.collision_avoidance = StopThenAvoidSystem()
        self.factory_floor = factory_floor
        
        # Statistics
        self.collision_events = 0
        self.near_miss_events = 0
        self.decision_count = 0
        
        self._prev_red_pairs: set = set()
        self._prev_yellow_pairs: set = set()
        
    def make_decisions(self, agv_fleet, current_time: float = 0) -> Dict[int, float]:
        """Make control decisions for all AGVs."""
        self.decision_count += 1
        
        # Compute avoidance (pass dt for deadlock timer)
        dt = cfg.SIMULATION_DT
        commands = self.collision_avoidance.update(agv_fleet, dt)
        
        # Apply commands to AGVs
        for agv in agv_fleet.agvs:
            if agv.id in commands:
                speed_factor, lateral_offset = commands[agv.id]
                agv.set_avoidance_command(speed_factor, lateral_offset)
        
        self._update_event_counts()
        return {agv_id: cmd[0] for agv_id, cmd in commands.items()}
    
    def _update_event_counts(self):
        """Track unique events."""
        current_red = set()
        current_yellow = set()
        
        for risk in self.collision_avoidance.active_risks:
            pair = (min(risk.agv1_id, risk.agv2_id), max(risk.agv1_id, risk.agv2_id))
            if risk.severity == SafetyZone.RED:
                current_red.add(pair)
            elif risk.severity == SafetyZone.YELLOW:
                current_yellow.add(pair)
        
        new_red = current_red - self._prev_red_pairs
        new_yellow = (current_yellow - self._prev_yellow_pairs) - current_red
        
        self.collision_events += len(new_red)
        self.near_miss_events += len(new_yellow)
        
        self._prev_red_pairs = current_red
        self._prev_yellow_pairs = current_yellow
    
    def get_predicted_trajectories(self, horizon: float = None) -> Dict[int, List[np.ndarray]]:
        """Get predicted trajectories for all AGVs."""
        horizon = horizon or cfg.PREDICTION_HORIZON
        positions = self.localization.get_all_estimates()
        velocities = self.localization.get_all_velocities()
        
        trajectories = {}
        dt = 0.1
        num_steps = int(horizon / dt)
        
        for agv_id in positions:
            traj = [positions[agv_id].copy()]
            pos = positions[agv_id].copy()
            vel = velocities[agv_id]
            
            for _ in range(num_steps):
                pos = pos + vel * dt
                pos[0] = np.clip(pos[0], 0, cfg.FACTORY_LENGTH)
                pos[1] = np.clip(pos[1], 0, cfg.FACTORY_WIDTH)
                traj.append(pos.copy())
            
            trajectories[agv_id] = traj
        return trajectories
    
    def get_statistics(self) -> Dict:
        """Get decision making statistics."""
        return {
            'total_decisions': self.decision_count,
            'collision_events': self.collision_events,  
            'near_miss_events': self.near_miss_events,
            'collision_rate': self.collision_events / max(1, self.decision_count),
            'near_miss_rate': self.near_miss_events / max(1, self.decision_count)
        }