"""
6G ISAC System Configuration
All physical parameters for the integrated sensing and communication system.
"""

import numpy as np

# =============================================================================
# PHYSICAL CONSTANTS
# =============================================================================
SPEED_OF_LIGHT = 3e8  # m/s

# =============================================================================
# 6G PHYSICAL LAYER PARAMETERS
# =============================================================================
CARRIER_FREQUENCY = 140e9       # 140 GHz (sub-THz 6G candidate)
BANDWIDTH = 2e9                  # 2 GHz bandwidth
SUBCARRIER_SPACING = 480e3       # 480 kHz (extended 5G NR)
NUM_SUBCARRIERS = 4096           # OFDM subcarriers
OFDM_SYMBOL_DURATION = 1 / SUBCARRIER_SPACING  # ~2.08 μs
TX_POWER_DBM = 30                # 30 dBm transmission power
TX_POWER_W = 10 ** ((TX_POWER_DBM - 30) / 10)  # Convert to Watts

# Antenna Array Configuration (Uniform Planar Array)
NUM_ANTENNA_X = 64               # Horizontal elements
NUM_ANTENNA_Y = 64               # Vertical elements
TOTAL_ANTENNAS = NUM_ANTENNA_X * NUM_ANTENNA_Y
ANTENNA_SPACING = 0.5 * (SPEED_OF_LIGHT / CARRIER_FREQUENCY)  # λ/2 spacing

# Derived Parameters
WAVELENGTH = SPEED_OF_LIGHT / CARRIER_FREQUENCY
RANGE_RESOLUTION = SPEED_OF_LIGHT / (2 * BANDWIDTH)  # ~7.5 cm
MAX_UNAMBIGUOUS_RANGE = SPEED_OF_LIGHT / (2 * SUBCARRIER_SPACING)  # ~312.5 m
ANGULAR_RESOLUTION_DEG = np.degrees(0.886 * WAVELENGTH / (NUM_ANTENNA_X * ANTENNA_SPACING))

# Noise Parameters
NOISE_FIGURE_DB = 8              # Receiver noise figure
THERMAL_NOISE_DBM = -174 + 10 * np.log10(BANDWIDTH) + NOISE_FIGURE_DB
THERMAL_NOISE_W = 10 ** ((THERMAL_NOISE_DBM - 30) / 10)

# =============================================================================
# FACTORY FLOOR GEOMETRY
# =============================================================================
FACTORY_LENGTH = 50.0            # meters (X-axis)
FACTORY_WIDTH = 30.0             # meters (Y-axis)
CEILING_HEIGHT = 8.0             # meters (Z-axis)

# Base Station Position (center of ceiling)
BS_POSITION = np.array([FACTORY_LENGTH / 2, FACTORY_WIDTH / 2, CEILING_HEIGHT])

# Storage Rack Positions (obstacles) - [x_min, y_min, x_max, y_max]
STORAGE_RACKS = [
    np.array([3, 5, 8, 10]),      # Rack 1 (bottom-left)
    np.array([42, 5, 47, 10]),    # Rack 2 (bottom-right)
    np.array([3, 20, 8, 25]),     # Rack 3 (top-left)
    np.array([42, 20, 47, 25]),   # Rack 4 (top-right)
]

# Charging Stations
CHARGING_STATIONS = [
    np.array([2, 15]),
    np.array([48, 15]),
]

# =============================================================================
# AGV SPECIFICATIONS
# =============================================================================
NUM_AGVS = 5
AGV_LENGTH = 1.2                 # meters
AGV_WIDTH = 0.8                  # meters
AGV_HEIGHT = 0.5                 # meters
AGV_MAX_VELOCITY = 2.0           # m/s
AGV_MAX_ACCELERATION = 0.5       # m/s²
AGV_MAX_ANGULAR_VELOCITY = np.radians(45)  # 45 deg/s
AGV_RCS = 1.0                    # Radar Cross Section (m²)

# Initial AGV Positions
AGV_INITIAL_POSITIONS = [
    np.array([10.0, 12.0]),
    np.array([20.0, 8.0]),
    np.array([35.0, 18.0]),
    np.array([25.0, 22.0]),
    np.array([15.0, 25.0]),
]

# =============================================================================
# CHANNEL MODEL PARAMETERS (3GPP Indoor Factory - LoS)
# =============================================================================
PATH_LOSS_EXPONENT = 2.15        # InF-SL (3GPP TR 38.901 standard, LOS)
SHADOW_FADING_STD_DB = 4.0       # dB
ATMOSPHERIC_ABSORPTION_DB_PER_KM = 2.0  # ITU-R P.676-13 at 140 GHz, standard atmosphere

# Multipath Parameters
NUM_MULTIPATH_CLUSTERS = 3
MULTIPATH_DELAY_SPREAD_NS = 50   # nanoseconds
MULTIPATH_POWER_DECAY_DB = 10    # dB per cluster

# =============================================================================
# LOCALIZATION PARAMETERS
# =============================================================================
# Measurement Noise Standard Deviations
RANGE_NOISE_STD = 0.05           # 5 cm range noise
ANGLE_NOISE_STD_DEG = 0.5        # 0.5 degree angle noise

# Extended Kalman Filter Parameters
EKF_PROCESS_NOISE_POS = 0.1      # Position process noise
EKF_PROCESS_NOISE_VEL = 0.5      # Velocity process noise

# =============================================================================
# DECISION MAKING PARAMETERS
# =============================================================================
PREDICTION_HORIZON = 3.0         # seconds
SAFETY_ZONE_RED = 1.5            # meters (emergency stop)
SAFETY_ZONE_YELLOW = 3.0         # meters (slow down)
SAFETY_ZONE_GREEN = 5.0          # meters (normal operation)
COLLISION_CHECK_INTERVAL = 0.1   # seconds

# =============================================================================
# SIMULATION PARAMETERS
# =============================================================================
SIMULATION_DT = 0.01             # 100 Hz update rate
SENSING_INTERVAL = 0.05          # 50 ms sensing cycle (20 Hz)
COMMUNICATION_INTERVAL = 0.001   # 1 ms communication slot

# Visualization
VIZ_UPDATE_RATE = 30             # 30 FPS for visualization
VIZ_TRAIL_LENGTH = 50            # Number of past positions to show
