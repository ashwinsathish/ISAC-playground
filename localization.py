"""
Localization System
Implements ToA/AoA estimation and Extended Kalman Filter tracking.
"""

import numpy as np
from typing import Tuple, List, Optional, Dict
import config as cfg
from channel_model import ChannelModel


class ToAAoAEstimator:
    """
    Estimates Time of Arrival (ToA) and Angle of Arrival (AoA) 
    from received signals.
    """
    
    def __init__(self, channel_model: ChannelModel):
        self.channel = channel_model
        self.range_resolution = cfg.RANGE_RESOLUTION
        self.bs_position = cfg.BS_POSITION
        
    def estimate_range(self, true_distance: float, snr_db: float) -> float:
        """
        Estimate range from ToA measurement.
        
        Args:
            true_distance: True 3D distance in meters
            snr_db: Signal-to-noise ratio in dB
            
        Returns:
            Estimated range in meters
        """
        # Add noise based on CRLB
        if snr_db > -10:  # Reasonable detection
            crlb = self.channel.get_cramer_rao_bound_range(snr_db)
            # Use slightly more noise than CRLB (practical estimator)
            noise_std = max(crlb * 1.5, cfg.RANGE_NOISE_STD)
            range_est = true_distance + np.random.normal(0, noise_std)
            return max(0.1, range_est)
        else:
            # Low SNR: return noisy estimate
            return true_distance + np.random.normal(0, 1.0)
    
    def estimate_angles(self, true_azimuth: float, true_elevation: float, 
                        snr_db: float) -> Tuple[float, float]:
        """
        Estimate azimuth and elevation angles.
        
        Args:
            true_azimuth: True azimuth angle in radians
            true_elevation: True elevation angle in radians
            snr_db: Signal-to-noise ratio in dB
            
        Returns:
            (estimated_azimuth, estimated_elevation) in radians
        """
        if snr_db > -10:
            crlb = self.channel.get_cramer_rao_bound_angle(snr_db)
            noise_std = max(crlb * 1.5, np.radians(cfg.ANGLE_NOISE_STD_DEG))
            
            az_est = true_azimuth + np.random.normal(0, noise_std)
            el_est = true_elevation + np.random.normal(0, noise_std)
            
            return az_est, el_est
        else:
            noise_std = np.radians(5.0)  # 5 degree noise at low SNR
            return (true_azimuth + np.random.normal(0, noise_std),
                    true_elevation + np.random.normal(0, noise_std))
    
    def estimate_position(self, range_est: float, azimuth: float, 
                         elevation: float) -> np.ndarray:
        """
        Estimate 3D position from range and angles.
        
        Args:
            range_est: Estimated range in meters
            azimuth: Estimated azimuth angle in radians
            elevation: Estimated elevation angle in radians
            
        Returns:
            Estimated [x, y, z] position
        """
        # Position relative to BS
        x_rel = range_est * np.cos(elevation) * np.cos(azimuth)
        y_rel = range_est * np.cos(elevation) * np.sin(azimuth)
        z_rel = range_est * np.sin(elevation)
        
        # Convert to global coordinates
        position = self.bs_position + np.array([x_rel, y_rel, z_rel])
        
        # Clamp to factory bounds
        position[0] = np.clip(position[0], 0, cfg.FACTORY_LENGTH)
        position[1] = np.clip(position[1], 0, cfg.FACTORY_WIDTH)
        position[2] = np.clip(position[2], 0, cfg.CEILING_HEIGHT)
        
        return position


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for AGV tracking.
    State: [x, y, vx, vy]
    """
    
    def __init__(self, initial_position: np.ndarray):
        # State vector: [x, y, vx, vy]
        self.state = np.array([
            initial_position[0], 
            initial_position[1], 
            0.0, 
            0.0
        ])
        
        # State covariance
        self.P = np.diag([1.0, 1.0, 0.5, 0.5])
        
        # Process noise
        self.Q = np.diag([
            cfg.EKF_PROCESS_NOISE_POS,
            cfg.EKF_PROCESS_NOISE_POS,
            cfg.EKF_PROCESS_NOISE_VEL,
            cfg.EKF_PROCESS_NOISE_VEL
        ])
        
        # Measurement noise (range, azimuth)
        self.R = np.diag([
            cfg.RANGE_NOISE_STD ** 2,
            np.radians(cfg.ANGLE_NOISE_STD_DEG) ** 2
        ])
        
        self.bs_position = cfg.BS_POSITION[:2]  # 2D base station position
        
    def predict(self, dt: float) -> np.ndarray:
        """
        Predict step of EKF.
        
        Args:
            dt: Time step in seconds
            
        Returns:
            Predicted state
        """
        # State transition matrix (constant velocity model)
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Predict state
        self.state = F @ self.state
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q * dt
        
        return self.state.copy()
    
    def update(self, range_meas: float, azimuth_meas: float) -> np.ndarray:
        """
        Update step of EKF with range and azimuth measurements.
        
        Args:
            range_meas: Measured range in meters
            azimuth_meas: Measured azimuth in radians
            
        Returns:
            Updated state
        """
        # Predicted measurement
        dx = self.state[0] - self.bs_position[0]
        dy = self.state[1] - self.bs_position[1]
        
        range_pred = np.sqrt(dx**2 + dy**2)
        azimuth_pred = np.arctan2(dy, dx)
        
        # Measurement Jacobian
        if range_pred < 0.1:
            range_pred = 0.1
            
        H = np.array([
            [dx / range_pred, dy / range_pred, 0, 0],
            [-dy / (range_pred**2), dx / (range_pred**2), 0, 0]
        ])
        
        # Innovation (measurement residual)
        z = np.array([range_meas, azimuth_meas])
        z_pred = np.array([range_pred, azimuth_pred])
        y = z - z_pred
        
        # Wrap angle residual
        while y[1] > np.pi:
            y[1] -= 2 * np.pi
        while y[1] < -np.pi:
            y[1] += 2 * np.pi
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        I = np.eye(4)
        self.P = (I - K @ H) @ self.P
        
        return self.state.copy()
    
    def get_position(self) -> np.ndarray:
        """Get current estimated position [x, y]."""
        return self.state[:2].copy()
    
    def get_velocity(self) -> np.ndarray:
        """Get current estimated velocity [vx, vy]."""
        return self.state[2:4].copy()
    
    def predict_future_position(self, horizon: float) -> np.ndarray:
        """
        Predict future position assuming constant velocity.
        
        Args:
            horizon: Prediction horizon in seconds
            
        Returns:
            Predicted [x, y] position
        """
        return self.state[:2] + self.state[2:4] * horizon


class LocalizationSystem:
    """
    Complete localization system managing multiple AGV tracks.
    """
    
    def __init__(self, num_agvs: int):
        self.channel = ChannelModel()
        self.estimator = ToAAoAEstimator(self.channel)
        
        # EKF for each AGV
        self.trackers: Dict[int, ExtendedKalmanFilter] = {}
        
        # Measurement history for each AGV
        self.measurement_history: Dict[int, List[dict]] = {}
        
        # Performance metrics
        self.position_errors: Dict[int, List[float]] = {}
        
        for i in range(num_agvs):
            # Initialize with approximate positions (generate if needed)
            if i < len(cfg.AGV_INITIAL_POSITIONS):
                init_pos = cfg.AGV_INITIAL_POSITIONS[i]
            else:
                # Generate random position for additional AGVs
                init_pos = np.array([
                    np.random.uniform(5, cfg.FACTORY_LENGTH - 5),
                    np.random.uniform(5, cfg.FACTORY_WIDTH - 5)
                ])
            self.trackers[i] = ExtendedKalmanFilter(init_pos)
            self.measurement_history[i] = []
            self.position_errors[i] = []
    
    def update(self, agv_id: int, true_position: np.ndarray, 
               true_velocity: np.ndarray, dt: float) -> Tuple[np.ndarray, float]:
        """
        Update localization for a single AGV.
        
        Args:
            agv_id: AGV identifier
            true_position: True [x, y] position (ground truth)
            true_velocity: True [vx, vy] velocity (ground truth)
            dt: Time step
            
        Returns:
            (estimated_position, localization_error)
        """
        tracker = self.trackers[agv_id]
        
        # Prediction step
        tracker.predict(dt)
        
        # Calculate true 3D position (AGV center height)
        true_pos_3d = np.array([true_position[0], true_position[1], 
                                cfg.AGV_HEIGHT / 2])
        
        # Calculate true range and angles from BS
        delta = true_pos_3d - cfg.BS_POSITION
        true_range = np.linalg.norm(delta)
        true_azimuth = np.arctan2(delta[1], delta[0])
        true_elevation = np.arctan2(delta[2], np.sqrt(delta[0]**2 + delta[1]**2))
        
        # Calculate SNR for this AGV
        snr_db = self.channel.calculate_radar_snr(true_range, cfg.AGV_RCS, agv_id)
        
        # Generate noisy measurements
        range_meas = self.estimator.estimate_range(true_range, snr_db)
        az_meas, el_meas = self.estimator.estimate_angles(
            true_azimuth, true_elevation, snr_db
        )
        
        # For 2D tracking, project range to horizontal plane
        horizontal_range = range_meas * np.cos(el_meas)
        
        # Update EKF with measurements
        tracker.update(horizontal_range, az_meas)
        
        # Get estimated position
        est_position = tracker.get_position()
        
        # Calculate error
        error = np.linalg.norm(est_position - true_position[:2])
        self.position_errors[agv_id].append(error)
        
        # Keep only recent errors
        if len(self.position_errors[agv_id]) > 100:
            self.position_errors[agv_id].pop(0)
        
        # Store measurement
        self.measurement_history[agv_id].append({
            'time': dt,
            'range': range_meas,
            'azimuth': az_meas,
            'snr_db': snr_db,
            'error': error
        })
        
        if len(self.measurement_history[agv_id]) > 50:
            self.measurement_history[agv_id].pop(0)
        
        return est_position, error
    
    def get_all_estimates(self) -> Dict[int, np.ndarray]:
        """Get estimated positions for all AGVs."""
        return {i: tracker.get_position() for i, tracker in self.trackers.items()}
    
    def get_all_velocities(self) -> Dict[int, np.ndarray]:
        """Get estimated velocities for all AGVs."""
        return {i: tracker.get_velocity() for i, tracker in self.trackers.items()}
    
    def predict_future_positions(self, horizon: float) -> Dict[int, np.ndarray]:
        """Predict future positions for all AGVs."""
        return {i: tracker.predict_future_position(horizon) 
                for i, tracker in self.trackers.items()}
    
    def get_average_error(self) -> float:
        """Get average localization error across all AGVs."""
        all_errors = []
        for errors in self.position_errors.values():
            all_errors.extend(errors)
        return np.mean(all_errors) if all_errors else 0.0
    
    def get_rmse(self) -> float:
        """Get RMS localization error."""
        all_errors = []
        for errors in self.position_errors.values():
            all_errors.extend(errors)
        if all_errors:
            return np.sqrt(np.mean(np.array(all_errors) ** 2))
        return 0.0
