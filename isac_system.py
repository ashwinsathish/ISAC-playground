"""
ISAC System
Integrates sensing and communication functionality.
"""

import numpy as np
from typing import Dict, Tuple, List
import config as cfg
from channel_model import ChannelModel, OFDMWaveform


class BaseStation:
    """
    6G Base Station with massive MIMO antenna array.
    Located at the center of the factory ceiling.
    """
    
    def __init__(self):
        self.position = cfg.BS_POSITION.copy()
        self.num_antennas_x = cfg.NUM_ANTENNA_X
        self.num_antennas_y = cfg.NUM_ANTENNA_Y
        self.total_antennas = cfg.TOTAL_ANTENNAS
        self.antenna_spacing = cfg.ANTENNA_SPACING
        self.wavelength = cfg.WAVELENGTH
        
        # Current beam directions (azimuth, elevation for each tracked AGV)
        self.beam_directions: Dict[int, Tuple[float, float]] = {}
        
        # Antenna array geometry (planar array on ceiling, facing down)
        self._create_antenna_array()
        
    def _create_antenna_array(self):
        """Create antenna element positions."""
        # Array centered at BS position
        x_positions = np.arange(self.num_antennas_x) * self.antenna_spacing
        y_positions = np.arange(self.num_antennas_y) * self.antenna_spacing
        
        x_positions -= np.mean(x_positions)
        y_positions -= np.mean(y_positions)
        
        self.antenna_positions = []
        for x in x_positions:
            for y in y_positions:
                self.antenna_positions.append(np.array([x, y, 0]))
        
        self.antenna_positions = np.array(self.antenna_positions)
        
    def calculate_beam_steering_vector(self, azimuth: float, elevation: float
                                       ) -> np.ndarray:
        """
        Calculate beam steering vector for given direction.
        
        Args:
            azimuth: Azimuth angle in radians
            elevation: Elevation angle in radians
            
        Returns:
            Complex steering vector
        """
        # Wave vector components
        kx = 2 * np.pi / self.wavelength * np.cos(elevation) * np.cos(azimuth)
        ky = 2 * np.pi / self.wavelength * np.cos(elevation) * np.sin(azimuth)
        
        # Phase shifts for each antenna
        phases = kx * self.antenna_positions[:, 0] + ky * self.antenna_positions[:, 1]
        
        steering_vector = np.exp(1j * phases)
        steering_vector /= np.linalg.norm(steering_vector)
        
        return steering_vector
    
    def steer_beam_to_agv(self, agv_id: int, agv_position: np.ndarray):
        """
        Steer a beam towards an AGV.
        
        Args:
            agv_id: AGV identifier
            agv_position: AGV [x, y] or [x, y, z] position
        """
        # Calculate direction to AGV
        if len(agv_position) == 2:
            agv_pos_3d = np.array([agv_position[0], agv_position[1], 
                                   cfg.AGV_HEIGHT / 2])
        else:
            agv_pos_3d = agv_position
            
        delta = agv_pos_3d - self.position
        
        # Calculate angles
        azimuth = np.arctan2(delta[1], delta[0])
        horizontal_dist = np.sqrt(delta[0]**2 + delta[1]**2)
        elevation = np.arctan2(-delta[2], horizontal_dist)  # Negative because looking down
        
        self.beam_directions[agv_id] = (azimuth, elevation)
    
    def get_beamforming_gain(self, agv_id: int, actual_direction: Tuple[float, float]
                            ) -> float:
        """
        Calculate beamforming gain for an AGV.
        
        Args:
            agv_id: AGV identifier
            actual_direction: Actual (azimuth, elevation) to AGV
            
        Returns:
            Gain in dB
        """
        if agv_id not in self.beam_directions:
            # Perfect beam steering assumed
            return 10 * np.log10(self.total_antennas)
        
        aimed_az, aimed_el = self.beam_directions[agv_id]
        actual_az, actual_el = actual_direction
        
        # Angular mismatch loss (simplified model)
        az_diff = aimed_az - actual_az
        el_diff = aimed_el - actual_el
        
        # Normalized angle difference
        theta_diff = np.sqrt(az_diff**2 + el_diff**2)
        
        # Beamwidth (approximate)
        beamwidth = self.wavelength / (self.num_antennas_x * self.antenna_spacing)
        
        # Gain reduction due to mismatch
        if theta_diff < beamwidth:
            gain_reduction = 0
        else:
            gain_reduction = min(20, 20 * (theta_diff / beamwidth - 1))
        
        max_gain = 10 * np.log10(self.total_antennas)
        return max_gain - gain_reduction


class ISACSystem:
    """
    Integrated Sensing and Communication System.
    Combines radar sensing with communication in a unified 6G system.
    """
    
    def __init__(self):
        self.base_station = BaseStation()
        self.channel = ChannelModel()
        self.waveform = OFDMWaveform()
        
        # Communication state per AGV
        self.comm_throughput: Dict[int, float] = {}
        self.sensing_snr: Dict[int, float] = {}
        
        # System statistics
        self.total_data_transmitted = 0  # bits
        self.sensing_updates = 0
        
    def sense_agvs(self, agv_positions: Dict[int, np.ndarray],
                   agv_velocities: Dict[int, np.ndarray]
                   ) -> Dict[int, Dict]:
        """
        Perform radar sensing on all AGVs.
        
        Args:
            agv_positions: Dict mapping AGV ID to position
            agv_velocities: Dict mapping AGV ID to velocity
            
        Returns:
            Dict mapping AGV ID to sensing results
        """
        self.sensing_updates += 1
        results = {}
        
        for agv_id, position in agv_positions.items():
            # Get 3D position
            if len(position) == 2:
                pos_3d = np.array([position[0], position[1], cfg.AGV_HEIGHT / 2])
            else:
                pos_3d = position
                
            # Calculate distance
            distance = np.linalg.norm(pos_3d - self.base_station.position)
            
            # Steer beam to AGV
            self.base_station.steer_beam_to_agv(agv_id, position)
            
            # Calculate radar SNR
            radar_snr = self.channel.calculate_radar_snr(distance, cfg.AGV_RCS, agv_id)
            self.sensing_snr[agv_id] = radar_snr
            
            # Calculate radial velocity
            velocity = agv_velocities.get(agv_id, np.zeros(2))
            delta = pos_3d - self.base_station.position
            unit_dir = delta / np.linalg.norm(delta)
            radial_velocity = np.dot(velocity, unit_dir[:2])
            
            # Estimate range and Doppler
            range_est, velocity_est = self.waveform.estimate_range_doppler(
                distance, radial_velocity, radar_snr
            )
            
            results[agv_id] = {
                'true_range': distance,
                'estimated_range': range_est,
                'range_error': abs(range_est - distance),
                'true_radial_velocity': radial_velocity,
                'estimated_velocity': velocity_est,
                'radar_snr_db': radar_snr,
                'detection': radar_snr > -5  # Detection threshold
            }
        
        return results
    
    def communicate_with_agvs(self, agv_positions: Dict[int, np.ndarray],
                              packet_size_bits: int = 1000
                              ) -> Dict[int, Dict]:
        """
        Perform downlink communication to all AGVs.
        
        Args:
            agv_positions: Dict mapping AGV ID to position
            packet_size_bits: Size of packet to send
            
        Returns:
            Dict mapping AGV ID to communication results
        """
        results = {}
        
        for agv_id, position in agv_positions.items():
            # Get 3D position
            if len(position) == 2:
                pos_3d = np.array([position[0], position[1], cfg.AGV_HEIGHT / 2])
            else:
                pos_3d = position
            
            # Calculate distance
            distance = np.linalg.norm(pos_3d - self.base_station.position)
            
            # Calculate SNR
            snr_db = self.channel.calculate_snr(distance, agv_id=agv_id)
            
            # Calculate achievable rate (Shannon capacity)
            if snr_db > -20:
                snr_linear = 10 ** (snr_db / 10)
                capacity_bps = cfg.BANDWIDTH * np.log2(1 + snr_linear)
            else:
                capacity_bps = 0
            
            # Throughput in Mbps
            throughput_mbps = capacity_bps / 1e6
            self.comm_throughput[agv_id] = throughput_mbps
            
            # Latency for packet
            if throughput_mbps > 0:
                latency_ms = (packet_size_bits / (capacity_bps)) * 1000
            else:
                latency_ms = float('inf')
            
            self.total_data_transmitted += packet_size_bits
            
            results[agv_id] = {
                'distance': distance,
                'snr_db': snr_db,
                'capacity_mbps': throughput_mbps,
                'latency_ms': latency_ms,
                'packet_delivered': latency_ms < 10  # 10ms deadline
            }
        
        return results
    
    def get_system_metrics(self) -> Dict:
        """Get overall ISAC system metrics."""
        avg_sensing_snr = np.mean(list(self.sensing_snr.values())) if self.sensing_snr else 0
        avg_throughput = np.mean(list(self.comm_throughput.values())) if self.comm_throughput else 0
        
        return {
            'sensing_updates': self.sensing_updates,
            'average_sensing_snr_db': avg_sensing_snr,
            'average_comm_throughput_mbps': avg_throughput,
            'total_data_gbits': self.total_data_transmitted / 1e9,
            'num_agvs_tracked': len(self.sensing_snr),
            'detection_rate': sum(1 for snr in self.sensing_snr.values() if snr > -5) / 
                             max(1, len(self.sensing_snr))
        }
    
    def get_beam_visualization_data(self) -> List[Tuple[np.ndarray, float, float]]:
        """
        Get beam data for visualization.
        
        Returns:
            List of (bs_position, azimuth, elevation) for each beam
        """
        beams = []
        for agv_id, (az, el) in self.base_station.beam_directions.items():
            beams.append((self.base_station.position, az, el))
        return beams
