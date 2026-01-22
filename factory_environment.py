"""
Factory Environment Model
Defines the factory floor geometry, obstacles, and waypoint generation.
"""

import numpy as np
from typing import List, Tuple, Optional
import config as cfg


class FactoryFloor:
    """
    Represents the industrial factory floor environment.
    """
    
    def __init__(self):
        self.length = cfg.FACTORY_LENGTH
        self.width = cfg.FACTORY_WIDTH
        self.height = cfg.CEILING_HEIGHT
        self.storage_racks = cfg.STORAGE_RACKS
        self.charging_stations = cfg.CHARGING_STATIONS
        
        # Create navigation grid for waypoint generation
        self.grid_resolution = 1.0  # 1 meter grid
        self._create_navigation_grid()
        
        # Define waypoints for AGV navigation
        self._define_waypoints()
    
    def _create_navigation_grid(self):
        """Create a grid marking free space vs obstacles."""
        nx = int(self.length / self.grid_resolution)
        ny = int(self.width / self.grid_resolution)
        self.nav_grid = np.ones((nx, ny), dtype=bool)  # True = free
        
        # Mark obstacles
        for rack in self.storage_racks:
            x_min = int(rack[0] / self.grid_resolution)
            y_min = int(rack[1] / self.grid_resolution)
            x_max = int(rack[2] / self.grid_resolution)
            y_max = int(rack[3] / self.grid_resolution)
            self.nav_grid[x_min:x_max, y_min:y_max] = False
        
        # Add boundary margin
        margin = int(1.0 / self.grid_resolution)
        self.nav_grid[:margin, :] = False
        self.nav_grid[-margin:, :] = False
        self.nav_grid[:, :margin] = False
        self.nav_grid[:, -margin:] = False
    
    def _define_waypoints(self):
        """Define navigation waypoints for AGVs."""
        # Key waypoints in the factory (aisles, junctions, picking stations)
        self.waypoints = [
            # Main horizontal aisle (center)
            np.array([10, 15]), np.array([20, 15]), np.array([30, 15]), np.array([40, 15]),
            # Upper horizontal path
            np.array([10, 27]), np.array([20, 27]), np.array([30, 27]), np.array([40, 27]),
            # Lower horizontal path
            np.array([10, 3]), np.array([20, 3]), np.array([30, 3]), np.array([40, 3]),
            # Vertical aisles
            np.array([15, 8]), np.array([15, 15]), np.array([15, 22]),
            np.array([25, 8]), np.array([25, 15]), np.array([25, 22]),
            np.array([35, 8]), np.array([35, 15]), np.array([35, 22]),
            # Picking stations near racks
            np.array([9, 7.5]), np.array([41, 7.5]),
            np.array([9, 22.5]), np.array([41, 22.5]),
        ]
    
    def is_position_valid(self, position: np.ndarray, margin: float = 0.5) -> bool:
        """
        Check if a position is valid (inside factory, not in obstacle).
        
        Args:
            position: [x, y] position to check
            margin: Safety margin around obstacles
            
        Returns:
            True if position is valid, False otherwise
        """
        x, y = position[0], position[1]
        
        # Check factory boundaries
        if x < margin or x > self.length - margin:
            return False
        if y < margin or y > self.width - margin:
            return False
        
        # Check obstacles with margin
        for rack in self.storage_racks:
            if (rack[0] - margin < x < rack[2] + margin and
                rack[1] - margin < y < rack[3] + margin):
                return False
        
        return True
    
    def get_random_waypoint(self, exclude_positions: Optional[List[np.ndarray]] = None,
                            min_distance: float = 3.0) -> np.ndarray:
        """
        Get a random valid waypoint for navigation.
        
        Args:
            exclude_positions: Positions to avoid (e.g., current AGV positions)
            min_distance: Minimum distance from excluded positions
            
        Returns:
            Valid waypoint position [x, y]
        """
        valid_waypoints = self.waypoints.copy()
        
        if exclude_positions:
            valid_waypoints = [
                wp for wp in valid_waypoints
                if all(np.linalg.norm(wp - pos[:2]) > min_distance 
                       for pos in exclude_positions)
            ]
        
        if valid_waypoints:
            return valid_waypoints[np.random.randint(len(valid_waypoints))].copy()
        else:
            # Fallback: generate random valid position
            for _ in range(100):
                x = np.random.uniform(2, self.length - 2)
                y = np.random.uniform(2, self.width - 2)
                pos = np.array([x, y])
                if self.is_position_valid(pos):
                    return pos
            return np.array([self.length / 2, self.width / 2])
    
    def check_line_of_sight(self, pos1: np.ndarray, pos2: np.ndarray) -> bool:
        """
        Check if there's clear line of sight between two positions.
        
        Args:
            pos1: First position [x, y] or [x, y, z]
            pos2: Second position [x, y] or [x, y, z]
            
        Returns:
            True if line of sight is clear
        """
        # Simple implementation: check if line intersects any obstacle
        p1 = pos1[:2]
        p2 = pos2[:2]
        
        for rack in self.storage_racks:
            if self._line_intersects_rect(p1, p2, rack):
                return False
        return True
    
    def _line_intersects_rect(self, p1: np.ndarray, p2: np.ndarray, 
                               rect: np.ndarray) -> bool:
        """Check if a line segment intersects a rectangle."""
        # Rectangle corners
        x_min, y_min, x_max, y_max = rect
        
        # Check all four edges of rectangle
        edges = [
            (np.array([x_min, y_min]), np.array([x_max, y_min])),  # Bottom
            (np.array([x_max, y_min]), np.array([x_max, y_max])),  # Right
            (np.array([x_max, y_max]), np.array([x_min, y_max])),  # Top
            (np.array([x_min, y_max]), np.array([x_min, y_min])),  # Left
        ]
        
        for edge_start, edge_end in edges:
            if self._segments_intersect(p1, p2, edge_start, edge_end):
                return True
        
        # Also check if line is completely inside rectangle
        if (x_min <= p1[0] <= x_max and y_min <= p1[1] <= y_max):
            return True
        if (x_min <= p2[0] <= x_max and y_min <= p2[1] <= y_max):
            return True
            
        return False
    
    def _segments_intersect(self, p1: np.ndarray, p2: np.ndarray,
                            p3: np.ndarray, p4: np.ndarray) -> bool:
        """Check if two line segments intersect using cross product method."""
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        
        return (ccw(p1, p3, p4) != ccw(p2, p3, p4) and 
                ccw(p1, p2, p3) != ccw(p1, p2, p4))
    
    def get_distance_to_bs(self, position: np.ndarray) -> float:
        """Calculate 3D distance from position to base station."""
        pos_3d = np.array([position[0], position[1], cfg.AGV_HEIGHT / 2])
        return np.linalg.norm(pos_3d - cfg.BS_POSITION)
    
    def get_angle_to_bs(self, position: np.ndarray) -> Tuple[float, float]:
        """
        Calculate azimuth and elevation angles to base station.
        
        Returns:
            (azimuth, elevation) in radians
        """
        pos_3d = np.array([position[0], position[1], cfg.AGV_HEIGHT / 2])
        delta = pos_3d - cfg.BS_POSITION
        
        # Azimuth: angle in XY plane from BS perspective
        azimuth = np.arctan2(delta[1], delta[0])
        
        # Elevation: angle from horizontal (negative since BS is above)
        horizontal_dist = np.sqrt(delta[0]**2 + delta[1]**2)
        elevation = np.arctan2(-delta[2], horizontal_dist)
        
        return azimuth, elevation
