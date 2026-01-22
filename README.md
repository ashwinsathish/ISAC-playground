# 6G ISAC Factory Floor Simulation

A comprehensive simulation of Integrated Sensing and Communication (ISAC) for autonomous factory floor operations using 6G sub-THz wireless technology.

---

## Table of Contents

1. [Core Concept](#core-concept)
2. [System Architecture](#system-architecture)
3. [Physical Layer Parameters](#physical-layer-parameters)
4. [Channel Model](#channel-model)
5. [AGV Specifications](#agv-specifications)
6. [Localization System](#localization-system)
7. [Collision Avoidance](#collision-avoidance)
8. [ISAC Operation](#isac-operation)
9. [User Interface](#user-interface)
10. [Running the Simulation](#running-the-simulation)

---

## Core Concept

### What is ISAC?

**Integrated Sensing and Communication (ISAC)** is a key enabler for 6G networks where the same waveform and infrastructure performs both:

- **Communication**: Data transmission to/from AGVs
- **Sensing (Radar)**: Localization and tracking of AGVs

This dual functionality enables real-time situational awareness on factory floors without requiring separate sensor systems.

### Why 6G Sub-THz?

The simulation uses **140 GHz** carrier frequency (sub-THz band) because:

- **Large bandwidth** (2 GHz) enables centimeter-level range resolution
- **Massive MIMO** arrays (64×64 = 4096 elements) provide precise beamforming
- **High path loss** at sub-THz is compensated by the large antenna gain

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    6G Base Station                          │
│              (Center of Ceiling, 8m height)                 │
│                                                             │
│   ┌─────────┐    ┌─────────┐    ┌─────────────────┐        │
│   │ Massive │    │  OFDM   │    │   Beamforming   │        │
│   │  MIMO   │◄──►│Waveform │◄──►│   Controller    │        │
│   │ 64×64   │    │Generator│    │                 │        │
│   └─────────┘    └─────────┘    └─────────────────┘        │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼ Wireless Channel
┌─────────────────────────────────────────────────────────────┐
│                    Factory Floor (50m × 30m)                │
│                                                             │
│   ┌─────┐              ┌─────┐              ┌─────┐        │
│   │Rack1│              │     │              │Rack2│        │
│   └─────┘              │     │              └─────┘        │
│                   AGV ◄┘     └► AGV                        │
│   ┌─────┐       ▲      ▼       ▲   ▼        ┌─────┐        │
│   │Rack3│       │     AGV      │            │Rack4│        │
│   └─────┘       │              │            └─────┘        │
│              Waypoints      Waypoints                      │
└─────────────────────────────────────────────────────────────┘
```

### Key Components

| Component | File | Purpose |
|-----------|------|---------|
| Configuration | `config.py` | All system parameters |
| Channel Model | `channel_model.py` | 3GPP InF path loss, radar equation |
| ISAC System | `isac_system.py` | Sensing + communication integration |
| Localization | `localization.py` | ToA/AoA estimation + EKF tracking |
| Decision Making | `decision_making.py` | Collision avoidance |
| AGV Model | `agv.py` | Kinematic simulation |
| Visualization | `visualization.py` | Real-time display |

---

## Physical Layer Parameters

### Carrier and Bandwidth

| Parameter | Value | Description |
|-----------|-------|-------------|
| Carrier Frequency | **140 GHz** | Sub-THz 6G candidate |
| Bandwidth | **2 GHz** | Wideband for high resolution |
| Wavelength (λ) | 2.14 mm | c / f_c |
| Subcarrier Spacing | 480 kHz | Extended 5G NR |
| OFDM Symbol Duration | 2.08 μs | 1 / Δf |

### Derived Resolutions

**Range Resolution:**
$$\Delta R = \frac{c}{2B} = \frac{3 \times 10^8}{2 \times 2 \times 10^9} = 7.5 \text{ cm}$$

**Maximum Unambiguous Range:**
$$R_{max} = \frac{c}{2 \Delta f} = \frac{3 \times 10^8}{2 \times 480 \times 10^3} = 312.5 \text{ m}$$

**Angular Resolution:**
$$\Delta \theta = \frac{0.886 \lambda}{N_x \cdot d} = \frac{0.886 \times 2.14mm}{64 \times 1.07mm} \approx 1.59°$$

### Antenna Array

| Parameter | Value |
|-----------|-------|
| Array Size | 64 × 64 elements |
| Total Antennas | 4096 |
| Element Spacing | λ/2 = 1.07 mm |
| Maximum Array Gain | 10 log₁₀(4096) = **36.1 dB** |

### Power Budget

| Parameter | Value |
|-----------|-------|
| TX Power | 30 dBm (1 W) |
| Noise Figure | 8 dB |
| Thermal Noise | -174 + 10 log₁₀(B) + NF = **-73 dBm** |

---

## Channel Model

Based on **3GPP TR 38.901 Indoor Factory (InF)** model adapted for sub-THz.

### Path Loss Model

$$PL_{dB} = 32.4 + 20 \log_{10}(f_c [GHz]) + 17.3 \log_{10}(d [m]) + X_\sigma + \alpha_{atm} \cdot d$$

Where:
- **f_c** = 140 GHz (carrier frequency)
- **d** = 3D distance in meters
- **X_σ** = Shadow fading ~ N(0, 4 dB)
- **α_atm** = 10 dB/km (atmospheric absorption at 140 GHz)
- Path loss exponent = 1.73 (InF-SL LoS)

### Received Power

$$P_{rx} = P_{tx} - PL + G_{BF}$$

Where:
- **P_tx** = 30 dBm
- **G_BF** = 36.1 dB (beamforming gain)

### Communication SNR

$$SNR_{comm} = P_{rx} - N_0 = P_{tx} - PL + G_{BF} - N_0$$

### Shannon Capacity

$$C = B \cdot \log_2(1 + SNR_{linear})$$

---

## Radar Sensing

### Radar Equation

$$SNR_{radar} = \frac{P_t \cdot G^2 \cdot \lambda^2 \cdot \sigma}{(4\pi)^3 \cdot R^4 \cdot P_n}$$

Where:
- **P_t** = 1 W (transmit power)
- **G** = 4096 (array gain, linear)
- **λ** = 2.14 mm (wavelength)
- **σ** = 1 m² (AGV radar cross section)
- **R** = distance to target
- **P_n** = thermal noise power

### Cramer-Rao Lower Bounds

**Range CRLB:**
$$CRLB_{range} = \frac{c}{2B\sqrt{2 \cdot SNR}}$$

**Angle CRLB:**
$$CRLB_{angle} = \frac{\lambda}{2\pi \cdot d \cdot N \cdot \sqrt{2 \cdot SNR}}$$

---

## AGV Specifications

| Parameter | Value |
|-----------|-------|
| Dimensions | 1.2m × 0.8m × 0.5m |
| Max Velocity | 2.0 m/s |
| Max Acceleration | 0.5 m/s² |
| Max Angular Velocity | 45°/s |
| Radar Cross Section | 1.0 m² |

### Motion Model

The AGV uses a **bicycle kinematic model**:

```
Position update:
  x(t+dt) = x(t) + v·cos(θ)·dt
  y(t+dt) = y(t) + v·sin(θ)·dt
  θ(t+dt) = θ(t) + ω·dt

where:
  v = current speed (m/s)
  θ = heading angle (rad)
  ω = angular velocity (rad/s)
```

### Waypoint Navigation

AGVs navigate between waypoints defined on the factory floor:
- Main horizontal aisles at y = 3m, 15m, 27m
- Vertical connectors at x = 15m, 25m, 35m
- Picking stations near storage racks

---

## Localization System

### ToA/AoA Estimation

1. **Time of Arrival (ToA)**: Measures signal round-trip time
   - Range = c × ToA / 2

2. **Angle of Arrival (AoA)**: Uses antenna array phase differences
   - Azimuth = arctan(Δy / Δx)
   - Elevation = arctan(-Δz / √(Δx² + Δy²))

### Extended Kalman Filter (EKF)

**State Vector:** x = [x, y, vx, vy]ᵀ

**Prediction Step:**
```
State transition matrix F:
┌           ┐
│ 1  0  dt 0│
│ 0  1  0 dt│
│ 0  0  1  0│
│ 0  0  0  1│
└           ┘

x̂⁻ = F · x̂
P⁻ = F · P · Fᵀ + Q
```

**Update Step:**
```
Measurement: z = [range, azimuth]

Innovation: y = z - h(x̂⁻)
Kalman Gain: K = P⁻ · Hᵀ · (H · P⁻ · Hᵀ + R)⁻¹
Updated State: x̂ = x̂⁻ + K · y
Updated Covariance: P = (I - K · H) · P⁻
```

**Noise Parameters:**
| Type | Value |
|------|-------|
| Range noise σ | 5 cm |
| Angle noise σ | 0.5° |
| Process noise (position) | 0.1 |
| Process noise (velocity) | 0.5 |

---

## Collision Avoidance

### Safety Zone Classification

| Zone | Distance | Action |
|------|----------|--------|
| 🟢 GREEN | > 5.0m | Normal operation |
| 🟡 YELLOW | 3.0-5.0m | Slow to 50% speed |
| 🔴 RED | < 1.5m | Emergency stop |

### Stop-Then-Avoid Algorithm

```
1. Detect collision risk (RED zone)
   → Both AGVs STOP immediately

2. Start deadlock timer (2 seconds)
   → AGVs remain stopped, conflict visible

3. After timeout:
   → Lower-ID AGV activates avoidance
   → Moves laterally (2m offset to the right)
   → Other AGV waits

4. Once clear (distance > 3m):
   → Both resume normal navigation
```

### Why This Approach?

- **Conservative**: Stops immediately for safety
- **Demonstrates conflict**: User can see the collision risk
- **Deterministic resolution**: Lower ID always moves first
- **Natural behavior**: Mimics "keep right" traffic rules

---

## ISAC Operation

### Timing

| Operation | Interval | Rate |
|-----------|----------|------|
| Physics update | 10 ms | 100 Hz |
| Sensing cycle | 50 ms | 20 Hz |
| Communication slot | 1 ms | 1000 Hz |
| Visualization | 33 ms | 30 FPS |

### Per-Frame Processing

```
For each simulation frame:
  1. Update AGV physics (position, velocity)
  2. ISAC System:
     a. Steer beams toward AGVs
     b. Perform radar sensing (range/Doppler)
     c. Calculate communication throughput
  3. Localization:
     a. ToA/AoA measurements with noise
     b. EKF predict/update cycle
  4. Decision Making:
     a. Check inter-AGV distances
     b. Classify safety zones
     c. Issue speed/avoidance commands
  5. Visualization update
```

---

## User Interface

### Startup Configuration Panel

The GUI provides interactive controls:

| Control | Range | Default |
|---------|-------|---------|
| Frequency | 1-1000 GHz | 140 GHz |
| Bandwidth | 0.1-10 GHz | 2 GHz |
| Number of AGVs | 1-10 | 5 |
| Duration | 10-120 s | 30 s |
| Scenario | Normal/Collision/Deadlock/etc. | Normal |

### Visualization Window

**Factory Floor View:**
- Color-coded AGVs with heading indicators
- Trail showing recent path
- Star markers for target waypoints
- Safety zone circles (green/yellow/red)

**Localization Performance Plot:**
- Real-time RMSE tracking (blue line)
- Historical error trend

**System Status Panel:**
- SENSING: AGVs tracked, detection rate, SNR
- LOCALIZATION: RMSE, range resolution
- COMMUNICATION: Throughput, total data
- SITUATIONAL AWARENESS: Collision risks, warnings

---

## Running the Simulation

### Installation

```bash
cd 6G_ISAC_Factory
python -m venv venv
source venv/bin/activate  # macOS/Linux
pip install -r requirements.txt
```

### Launch Options

```bash
# Interactive GUI (default)
python main.py

# Skip config panel, direct visualization  
python main.py --no-gui

# Specific test scenario
python main.py --scenario collision

# Headless test mode
python main.py --test-mode --duration 30

# No visualization
python main.py --no-viz --duration 60
```

### Test Scenarios

| Scenario | Description |
|----------|-------------|
| `collision` | Two AGVs on direct collision course |
| `deadlock` | Multiple AGVs meet at intersection |
| `threeway` | Three-way conflict |
| `crossing` | Perpendicular crossing paths |

---

## File Structure

```
6G_ISAC_Factory/
├── main.py              # Entry point, simulation loop
├── config.py            # All system parameters
├── channel_model.py     # Path loss, radar equation, OFDM
├── isac_system.py       # Sensing + communication
├── localization.py      # ToA/AoA + EKF tracking
├── decision_making.py   # Collision avoidance
├── agv.py               # AGV kinematics
├── factory_environment.py  # Floor layout, obstacles
├── visualization.py     # Real-time display
├── control_panel.py     # Startup config GUI
├── test_scenarios.py    # Predefined test cases
└── requirements.txt     # Dependencies
```

---

## Key Formulas Summary

| Quantity | Formula |
|----------|---------|
| Range Resolution | Δr = c / (2B) |
| Path Loss (dB) | PL = 32.4 + 20log(f_GHz) + 17.3log(d) |
| Radar SNR | SNR = (Pt·G²·λ²·σ) / ((4π)³·R⁴·Pn) |
| Shannon Capacity | C = B·log₂(1 + SNR) |
| Range CRLB | σ_r = c / (2B√(2·SNR)) |
| Angle CRLB | σ_θ = λ / (2πdN√(2·SNR)) |
| EKF State | x = [x, y, vx, vy]ᵀ |

---

## Performance Metrics

| Metric | Typical Value |
|--------|---------------|
| Localization RMSE | 5-15 cm |
| Sensing SNR | 20-35 dB |
| Communication Throughput | 500-1500 Mbps |
| Detection Rate | 100% |

---

*This simulation demonstrates the potential of 6G ISAC technology for industrial automation, combining wireless communication with radar sensing in a single unified system.*
