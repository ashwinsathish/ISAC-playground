"""
6G Channel Model
Implements 3GPP Indoor Factory channel model adapted for sub-THz frequencies.
"""

import numpy as np
from typing import Tuple, List
import config as cfg


class ChannelModel:
    """
    6G sub-THz channel model for indoor factory environment.
    Based on 3GPP TR 38.901 Indoor Factory model with sub-THz extensions.
    """
    
    def __init__(self):
        self.carrier_freq = cfg.CARRIER_FREQUENCY
        self.wavelength = cfg.WAVELENGTH
        self.path_loss_exp = cfg.PATH_LOSS_EXPONENT
        self.shadow_fading_std = cfg.SHADOW_FADING_STD_DB
        self.atm_absorption = cfg.ATMOSPHERIC_ABSORPTION_DB_PER_KM
        
        # Cache for shadow fading (consistent per AGV)
        self.shadow_fading_cache = {}
        
    def calculate_path_loss(self, distance: float, include_fading: bool = True,
                           agv_id: int = None) -> float:
        """
        Calculate total path loss in dB.
        
        PL = 32.4 + 20*log10(fc_GHz) + 17.3*log10(d_m) + X_σ + α_atm*d
        
        Args:
            distance: 3D distance in meters
            include_fading: Whether to include shadow fading
            agv_id: AGV ID for consistent shadow fading
            
        Returns:
            Path loss in dB
        """
        if distance < 0.1:
            distance = 0.1  # Minimum distance to avoid log(0)
            
        # Free space + InF path loss
        fc_ghz = self.carrier_freq / 1e9
        pl_db = 32.4 + 20 * np.log10(fc_ghz) + self.path_loss_exp * 10 * np.log10(distance)
        
        # Atmospheric absorption (significant at sub-THz)
        atm_loss = self.atm_absorption * distance / 1000  # Convert m to km
        pl_db += atm_loss
        
        # Shadow fading
        if include_fading:
            if agv_id is not None:
                if agv_id not in self.shadow_fading_cache:
                    self.shadow_fading_cache[agv_id] = np.random.normal(0, self.shadow_fading_std)
                sf = self.shadow_fading_cache[agv_id]
            else:
                sf = np.random.normal(0, self.shadow_fading_std)
            pl_db += sf
            
        return pl_db
    
    def calculate_received_power(self, distance: float, tx_power_dbm: float = None,
                                 agv_id: int = None) -> float:
        """
        Calculate received signal power.
        
        Args:
            distance: 3D distance in meters
            tx_power_dbm: Transmit power in dBm
            agv_id: AGV ID for consistent channel
            
        Returns:
            Received power in dBm
        """
        if tx_power_dbm is None:
            tx_power_dbm = cfg.TX_POWER_DBM
            
        pl_db = self.calculate_path_loss(distance, agv_id=agv_id)
        
        # Add antenna gain (assuming beamforming gain)
        # For 64x64 array with ideal beamforming: G = 10*log10(N) ≈ 38 dB
        beamforming_gain = 10 * np.log10(cfg.TOTAL_ANTENNAS)
        
        rx_power_dbm = tx_power_dbm - pl_db + beamforming_gain
        
        return rx_power_dbm
    
    def calculate_snr(self, distance: float, agv_id: int = None) -> float:
        """
        Calculate Signal-to-Noise Ratio.
        
        Args:
            distance: 3D distance in meters
            agv_id: AGV ID for consistent channel
            
        Returns:
            SNR in dB
        """
        rx_power_dbm = self.calculate_received_power(distance, agv_id=agv_id)
        snr_db = rx_power_dbm - cfg.THERMAL_NOISE_DBM
        
        return snr_db
    
    def calculate_radar_snr(self, distance: float, rcs: float, 
                            agv_id: int = None) -> float:
        """
        Calculate radar SNR for sensing.
        
        Radar equation: SNR = (Pt * G^2 * λ^2 * σ) / ((4π)^3 * R^4 * Pn)
        
        Args:
            distance: Distance to target in meters
            rcs: Radar cross section in m²
            agv_id: AGV ID for consistent channel
            
        Returns:
            Radar SNR in dB
        """
        if distance < 0.1:
            distance = 0.1
            
        # Antenna gain
        G_db = 10 * np.log10(cfg.TOTAL_ANTENNAS)
        G = 10 ** (G_db / 10)
        
        # Radar equation in linear
        pt = cfg.TX_POWER_W
        wavelength = self.wavelength
        pn = cfg.THERMAL_NOISE_W
        
        snr_linear = (pt * G**2 * wavelength**2 * rcs) / \
                     ((4 * np.pi)**3 * distance**4 * pn)
        
        # Add path loss effects (atmospheric, etc.)
        extra_loss_db = self.atm_absorption * 2 * distance / 1000  # Two-way
        
        snr_db = 10 * np.log10(snr_linear) - extra_loss_db
        
        # Add some random variation
        if agv_id is not None:
            np.random.seed(agv_id + int(distance * 100))
        snr_db += np.random.normal(0, 2)  # 2 dB variation
        
        return snr_db
    
    def generate_multipath(self, distance: float, num_clusters: int = None
                          ) -> List[Tuple[float, float, float]]:
        """
        Generate multipath components.
        
        Args:
            distance: Direct path distance
            num_clusters: Number of multipath clusters
            
        Returns:
            List of (delay, power_dB, doppler) tuples
        """
        if num_clusters is None:
            num_clusters = cfg.NUM_MULTIPATH_CLUSTERS
            
        paths = []
        
        # Direct path (LoS)
        los_delay = distance / cfg.SPEED_OF_LIGHT
        paths.append((los_delay, 0.0, 0.0))  # Reference power = 0 dB
        
        # NLOS paths
        for i in range(num_clusters):
            # Excess delay
            excess_delay = np.random.exponential(cfg.MULTIPATH_DELAY_SPREAD_NS * 1e-9)
            path_delay = los_delay + excess_delay
            
            # Power decay with delay
            power_db = -cfg.MULTIPATH_POWER_DECAY_DB * (i + 1) + np.random.normal(0, 2)
            
            # Random Doppler (from reflector motion or BS phase noise)
            doppler = np.random.normal(0, 10)  # ±10 Hz variation
            
            paths.append((path_delay, power_db, doppler))
            
        return paths
    
    def get_cramer_rao_bound_range(self, snr_db: float) -> float:
        """
        Calculate Cramer-Rao Lower Bound for range estimation.
        
        CRLB = c / (2 * B * sqrt(2 * SNR))
        
        Args:
            snr_db: SNR in dB
            
        Returns:
            CRLB for range in meters
        """
        snr_linear = 10 ** (snr_db / 10)
        crlb = cfg.SPEED_OF_LIGHT / (2 * cfg.BANDWIDTH * np.sqrt(2 * snr_linear))
        return crlb
    
    def get_cramer_rao_bound_angle(self, snr_db: float) -> float:
        """
        Calculate Cramer-Rao Lower Bound for angle estimation.
        
        Args:
            snr_db: SNR in dB
            
        Returns:
            CRLB for angle in radians
        """
        snr_linear = 10 ** (snr_db / 10)
        # Simplified formula for ULA
        N = cfg.NUM_ANTENNA_X
        d = cfg.ANTENNA_SPACING
        
        crlb = self.wavelength / (2 * np.pi * d * N * np.sqrt(2 * snr_linear))
        return crlb


class OFDMWaveform:
    """
    OFDM-based ISAC waveform generator and processor.
    """
    
    def __init__(self):
        self.num_subcarriers = cfg.NUM_SUBCARRIERS
        self.subcarrier_spacing = cfg.SUBCARRIER_SPACING
        self.bandwidth = cfg.BANDWIDTH
        self.symbol_duration = cfg.OFDM_SYMBOL_DURATION
        
    def generate_sensing_signal(self, num_symbols: int = 14) -> np.ndarray:
        """
        Generate OFDM sensing signal (pilot symbols).
        
        Args:
            num_symbols: Number of OFDM symbols per frame
            
        Returns:
            Time-frequency grid of pilot symbols
        """
        # Use constant amplitude zero autocorrelation (CAZAC) sequence
        # Zadoff-Chu sequence for pilots
        root = 1
        pilot_sequence = np.exp(-1j * np.pi * root * 
                                np.arange(self.num_subcarriers) * 
                                (np.arange(self.num_subcarriers) + 1) / 
                                self.num_subcarriers)
        
        # Repeat for all symbols
        signal = np.tile(pilot_sequence, (num_symbols, 1))
        
        return signal
    
    def process_radar_echo(self, tx_signal: np.ndarray, rx_signal: np.ndarray
                          ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Process received radar echo to extract range-Doppler map.
        
        Args:
            tx_signal: Transmitted signal (symbols x subcarriers)
            rx_signal: Received signal (symbols x subcarriers)
            
        Returns:
            (range_axis, doppler_axis, range_doppler_map)
        """
        num_symbols, num_sc = tx_signal.shape
        
        # Channel estimation in frequency domain
        H = rx_signal / (tx_signal + 1e-10)
        
        # Range profile via IFFT across subcarriers
        range_profile = np.fft.ifft(H, axis=1)
        
        # Doppler profile via FFT across symbols
        range_doppler = np.fft.fft(range_profile, axis=0)
        
        # Axes
        range_axis = np.arange(num_sc) * cfg.RANGE_RESOLUTION
        doppler_resolution = 1 / (num_symbols * self.symbol_duration)
        doppler_axis = (np.arange(num_symbols) - num_symbols // 2) * doppler_resolution
        
        return range_axis, doppler_axis, np.abs(range_doppler)
    
    def estimate_range_doppler(self, distance: float, velocity: float,
                               snr_db: float) -> Tuple[float, float]:
        """
        Estimate range and Doppler with noise.
        
        Args:
            distance: True distance in meters
            velocity: True radial velocity in m/s
            snr_db: SNR in dB
            
        Returns:
            (estimated_range, estimated_doppler)
        """
        snr_linear = 10 ** (snr_db / 10)
        
        # Range estimation with CRLB-based noise
        range_std = cfg.RANGE_RESOLUTION / np.sqrt(2 * snr_linear)
        range_est = distance + np.random.normal(0, range_std)
        
        # Doppler estimation
        true_doppler = 2 * velocity * cfg.CARRIER_FREQUENCY / cfg.SPEED_OF_LIGHT
        doppler_std = 1 / (0.01 * np.sqrt(2 * snr_linear))  # Assuming 10ms frame
        doppler_est = true_doppler + np.random.normal(0, doppler_std)
        
        # Convert Doppler back to velocity
        velocity_est = doppler_est * cfg.SPEED_OF_LIGHT / (2 * cfg.CARRIER_FREQUENCY)
        
        return max(0, range_est), velocity_est
