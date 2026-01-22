"""
6G ISAC Factory Simulation - Main Entry Point

This simulation demonstrates:
1. Situational Awareness: Real-time detection and tracking of all AGVs
2. Localization: High-precision positioning using 6G sensing
3. Predictive Decision Making: Trajectory prediction and collision avoidance

Usage:
    python main.py                       # Run interactive simulation
    python main.py --duration 60         # Run for 60 seconds
    python main.py --test-mode           # Run verification tests
    python main.py --no-viz              # Run without visualization
    python main.py --scenario collision  # Run collision test scenario
    python main.py --scenario deadlock   # Run deadlock test scenario
    python main.py --list-scenarios      # List all available scenarios
"""

import numpy as np
import argparse
import time
import sys
from typing import Generator, Dict

import config as cfg
from factory_environment import FactoryFloor
from agv import AGVFleet
from channel_model import ChannelModel
from localization import LocalizationSystem
from decision_making import DecisionMaker
from isac_system import ISACSystem


def create_simulation():
    """Create and initialize all simulation components."""
    print("=" * 60)
    print("6G ISAC System - Industrial Factory Floor Simulation")
    print("=" * 60)
    print(f"\nSystem Parameters:")
    print(f"  • Carrier Frequency: {cfg.CARRIER_FREQUENCY/1e9:.0f} GHz")
    print(f"  • Bandwidth: {cfg.BANDWIDTH/1e9:.1f} GHz")
    print(f"  • Range Resolution: {cfg.RANGE_RESOLUTION*100:.1f} cm")
    print(f"  • Antenna Array: {cfg.NUM_ANTENNA_X}×{cfg.NUM_ANTENNA_Y} ({cfg.TOTAL_ANTENNAS} elements)")
    print(f"  • Factory Size: {cfg.FACTORY_LENGTH}m × {cfg.FACTORY_WIDTH}m")
    print(f"  • Number of AGVs: {cfg.NUM_AGVS}")
    print()
    
    # Initialize components
    print("Initializing simulation components...")
    factory = FactoryFloor()
    agv_fleet = AGVFleet(factory)
    isac = ISACSystem()
    localization = LocalizationSystem(cfg.NUM_AGVS)
    decision_maker = DecisionMaker(localization, factory_floor=factory)
    
    print("  ✓ Factory environment initialized")
    print("  ✓ AGV fleet initialized")
    print("  ✓ ISAC system initialized")
    print("  ✓ Localization system initialized")
    print("  ✓ Decision making system initialized")
    print()
    
    return {
        'factory': factory,
        'agv_fleet': agv_fleet,
        'isac': isac,
        'localization': localization,
        'decision_maker': decision_maker
    }


def run_simulation_step(components: Dict, dt: float, current_time: float) -> Dict:
    """
    Run one simulation step.
    
    Args:
        components: Dict of simulation components
        dt: Time step
        current_time: Current simulation time
        
    Returns:
        Dict containing frame data for visualization
    """
    factory = components['factory']
    agv_fleet = components['agv_fleet']
    isac = components['isac']
    localization = components['localization']
    decision_maker = components['decision_maker']
    
    # 1. Update AGV physics
    agv_fleet.update(dt)
    
    # 2. Get true AGV positions and velocities
    true_positions = {}
    true_velocities = {}
    for agv in agv_fleet.agvs:
        true_positions[agv.id] = agv.position.copy()
        true_velocities[agv.id] = agv.velocity.copy()
    
    # 3. ISAC Sensing
    sensing_results = isac.sense_agvs(true_positions, true_velocities)
    
    # 4. Communication (background, for metrics)
    comm_results = isac.communicate_with_agvs(true_positions)
    
    # 5. Localization update
    for agv_id, true_pos in true_positions.items():
        true_vel = true_velocities[agv_id]
        localization.update(agv_id, true_pos, true_vel, dt)
    
    # 6. Decision Making (pass current_time for deadlock resolution timing)
    speed_commands = decision_maker.make_decisions(agv_fleet, current_time)
    
    # 7. Get predicted trajectories
    predictions = decision_maker.get_predicted_trajectories(horizon=2.0)
    
    # 8. Compile frame data
    frame_data = {
        'time': current_time,
        'estimates': localization.get_all_estimates(),
        'predictions': predictions,
        'rmse': localization.get_rmse(),
        'awareness': decision_maker.collision_avoidance.get_situational_awareness(),
        'isac_metrics': isac.get_system_metrics(),
        'decision_stats': decision_maker.get_statistics(),
        'speed_commands': speed_commands,
        'sensing_results': sensing_results
    }
    
    return frame_data


def simulation_generator(components: Dict, duration: float = None, 
                        dt: float = None) -> Generator[Dict, None, None]:
    """
    Generator that yields frame data for each simulation step.
    
    Args:
        components: Simulation components
        duration: Total simulation duration in seconds (None = infinite)
        dt: Time step in seconds
        
    Yields:
        Frame data dict for each step
    """
    dt = dt or cfg.SIMULATION_DT
    current_time = 0.0
    step = 0
    sensing_interval = cfg.SENSING_INTERVAL
    last_sensing_time = 0.0
    
    while duration is None or current_time < duration:
        # Run physics at high rate
        frame_data = run_simulation_step(components, dt, current_time)
        
        # Yield data at visualization rate
        if current_time - last_sensing_time >= sensing_interval:
            last_sensing_time = current_time
            yield frame_data
        
        current_time += dt
        step += 1


def run_test_mode(components: Dict, duration: float = 30.0):
    """
    Run simulation in test mode and verify performance.
    
    Args:
        components: Simulation components
        duration: Test duration in seconds
    """
    print("Running in TEST MODE...")
    print(f"Test duration: {duration} seconds\n")
    
    dt = cfg.SIMULATION_DT
    current_time = 0.0
    
    errors = []
    detection_count = 0
    total_sensing = 0
    collision_warnings = 0
    
    start_real_time = time.time()
    
    while current_time < duration:
        frame_data = run_simulation_step(components, dt, current_time)
        
        # Collect metrics every 100ms
        if int(current_time * 10) % 1 == 0:
            errors.append(frame_data['rmse'])
            
            # Check detection rate
            sensing = frame_data['sensing_results']
            total_sensing += len(sensing)
            detection_count += sum(1 for r in sensing.values() if r['detection'])
            
            # Check collision warnings
            awareness = frame_data['awareness']
            if awareness['red_zone_count'] > 0:
                collision_warnings += 1
        
        current_time += dt
        
        # Progress update
        if int(current_time) % 5 == 0 and int((current_time - dt) / 1) != int(current_time / 1):
            print(f"  Progress: {current_time:.0f}/{duration:.0f}s | RMSE: {frame_data['rmse']*100:.2f} cm")
    
    elapsed_real = time.time() - start_real_time
    
    # Report results
    print("\n" + "=" * 60)
    print("TEST RESULTS")
    print("=" * 60)
    
    avg_rmse = np.mean(errors)
    max_rmse = np.max(errors)
    detection_rate = detection_count / max(1, total_sensing)
    
    print(f"\n📍 Localization Performance:")
    print(f"   • Average RMSE: {avg_rmse*100:.2f} cm")
    print(f"   • Maximum RMSE: {max_rmse*100:.2f} cm")
    print(f"   • Target: < 20 cm")
    print(f"   • Status: {'✅ PASS' if avg_rmse < 0.20 else '❌ FAIL'}")
    
    print(f"\n📡 Sensing Performance:")
    print(f"   • Detection Rate: {detection_rate*100:.1f}%")
    print(f"   • Status: {'✅ PASS' if detection_rate > 0.95 else '❌ FAIL'}")
    
    print(f"\n🚗 Collision Avoidance:")
    print(f"   • Red Zone Warnings: {collision_warnings}")
    print(f"   • Status: {'✅ System Active' if collision_warnings < duration else '⚠️ High Warning Rate'}")
    
    print(f"\n⚡ Performance:")
    print(f"   • Simulated Time: {duration:.1f}s")
    print(f"   • Real Time: {elapsed_real:.1f}s")
    print(f"   • Speed: {duration/elapsed_real:.1f}x real-time")
    
    # Overall pass/fail
    overall_pass = avg_rmse < 0.20 and detection_rate > 0.95
    print(f"\n{'='*60}")
    print(f"OVERALL: {'✅ ALL TESTS PASSED' if overall_pass else '❌ SOME TESTS FAILED'}")
    print("=" * 60)
    
    return overall_pass


def run_no_viz(components: Dict, duration: float = 60.0):
    """Run simulation without visualization."""
    print(f"Running simulation for {duration} seconds (no visualization)...\n")
    
    gen = simulation_generator(components, duration=duration)
    
    last_print_time = 0
    for frame_data in gen:
        current_time = frame_data['time']
        
        # Print status every 5 seconds
        if current_time - last_print_time >= 5.0:
            rmse = frame_data['rmse']
            awareness = frame_data['awareness']
            metrics = frame_data['isac_metrics']
            
            print(f"[{current_time:6.1f}s] RMSE: {rmse*100:5.2f}cm | "
                  f"Risks: {awareness['total_risks']} | "
                  f"SNR: {metrics['average_sensing_snr_db']:5.1f}dB | "
                  f"Throughput: {metrics['average_comm_throughput_mbps']:.0f}Mbps")
            
            last_print_time = current_time
    
    print("\nSimulation complete!")


def run_interactive(components: Dict, duration: float = None):
    """Run simulation with interactive visualization."""
    from visualization import FactoryVisualizer, SimulationRenderer
    
    print("Starting interactive visualization...")
    print("Close the window to end simulation.\n")
    
    # Create visualizer
    visualizer = FactoryVisualizer(
        components['factory'],
        components['agv_fleet'],
        components['isac'],
        components['localization'],
        components['decision_maker']
    )
    
    # Create renderer
    renderer = SimulationRenderer(visualizer)
    
    # Create simulation generator
    gen = simulation_generator(components, duration=duration)
    
    # Start animation
    try:
        renderer.start(gen)
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        visualizer.close()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='6G ISAC Factory Floor Simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py                    # Launch interactive GUI
  python main.py --no-gui           # Run with visualization (no GUI panel)
  python main.py --duration 30      # Run for 30 seconds
  python main.py --test-mode        # Run automated tests
  python main.py --no-viz           # Run without visualization
  python main.py --scenario collision  # Run specific test scenario
        """
    )
    
    parser.add_argument('--duration', type=float, default=None,
                       help='Simulation duration in seconds')
    parser.add_argument('--test-mode', action='store_true',
                       help='Run in test mode (verify performance)')
    parser.add_argument('--no-viz', action='store_true',
                       help='Run without visualization')
    parser.add_argument('--no-gui', action='store_true',
                       help='Run without the control panel GUI')
    parser.add_argument('--scenario', type=str, default=None,
                       help='Run a specific test scenario (collision, deadlock, threeway, crossing)')
    parser.add_argument('--list-scenarios', action='store_true',
                       help='List available test scenarios')
    
    args = parser.parse_args()
    
    # Handle --list-scenarios
    if args.list_scenarios:
        from test_scenarios import list_scenarios
        list_scenarios()
        sys.exit(0)
    
    # Launch GUI mode (default) or traditional mode
    if not args.no_gui and not args.test_mode and not args.no_viz and not args.scenario:
        # Launch interactive GUI
        from control_panel import ControlPanel
        run_gui_simulation()
    elif args.scenario:
        # Create simulation with scenario
        from test_scenarios import create_scenario_simulation
        components = create_scenario_simulation(args.scenario)
        run_interactive(components, args.duration)
    elif args.test_mode:
        components = create_simulation()
        duration = args.duration or 30.0
        success = run_test_mode(components, duration)
        sys.exit(0 if success else 1)
    elif args.no_viz:
        components = create_simulation()
        duration = args.duration or 60.0
        run_no_viz(components, duration)
    else:
        # --no-gui was specified
        components = create_simulation()
        run_interactive(components, args.duration)


def run_gui_simulation():
    """
    Run simulation with interactive GUI control panel.
    Uses a two-window approach: control panel + existing visualization.
    """
    import matplotlib.pyplot as plt
    from control_panel import ControlPanel
    
    # Create control panel
    panel = ControlPanel()
    
    def on_start(config):
        """Handle start button - create and launch simulation in separate window."""
        print(f"\n[START] Launching simulation with config:")
        print(f"   Scenario: {config['scenario']}")
        print(f"   AGVs: {config['num_agvs']}")
        print(f"   Duration: {config['duration']}s")
        print(f"   Frequency: {config['frequency']} GHz")
        
        # Close the control panel
        plt.close(panel.fig)
        
        # Update config with user's parameters
        cfg.NUM_AGVS = config['num_agvs']
        
        # Create simulation based on scenario
        if config['scenario'] == 'normal':
            # Create with user's AGV count
            components = create_simulation_with_agv_count(config['num_agvs'])
        else:
            from test_scenarios import create_scenario_simulation
            components = create_scenario_simulation(config['scenario'])
            # Reinitialize localization with correct AGV count
            from localization import LocalizationSystem
            num_agvs = len(components['agv_fleet'].agvs)
            components['localization'] = LocalizationSystem(num_agvs)
            components['decision_maker'].localization = components['localization']
        
        # Run with existing visualization system
        run_interactive(components, config['duration'])
    
    def on_pause(is_paused):
        print(f"   {'Paused' if is_paused else 'Resumed'}")
    
    def on_stop():
        print("   Stopped")
        plt.close('all')
    
    # Connect callbacks
    panel.on_start = on_start
    panel.on_pause = on_pause
    panel.on_stop = on_stop
    
    # Show control panel and wait for user to click PLAY
    panel.show()


def create_simulation_with_agv_count(num_agvs: int):
    """Create simulation components with specific AGV count."""
    print("=" * 60)
    print("6G ISAC System - Industrial Factory Floor Simulation")
    print("=" * 60)
    print(f"\nSystem Parameters:")
    print(f"  - Carrier Frequency: {cfg.CARRIER_FREQUENCY/1e9:.0f} GHz")
    print(f"  - Bandwidth: {cfg.BANDWIDTH/1e9:.1f} GHz")
    print(f"  - Range Resolution: {cfg.RANGE_RESOLUTION*100:.1f} cm")
    print(f"  - Antenna Array: {cfg.NUM_ANTENNA_X}x{cfg.NUM_ANTENNA_Y} ({cfg.TOTAL_ANTENNAS} elements)")
    print(f"  - Factory Size: {cfg.FACTORY_LENGTH}m x {cfg.FACTORY_WIDTH}m")
    print(f"  - Number of AGVs: {num_agvs}")
    print()
    
    # Initialize components
    print("Initializing simulation components...")
    
    # Temporarily override NUM_AGVS for AGVFleet
    original_num_agvs = cfg.NUM_AGVS
    cfg.NUM_AGVS = num_agvs
    
    factory = FactoryFloor()
    agv_fleet = AGVFleet(factory, num_agvs=num_agvs)  # Pass num_agvs explicitly
    isac = ISACSystem()
    
    # Localization with actual fleet size
    localization = LocalizationSystem(len(agv_fleet.agvs))
    decision_maker = DecisionMaker(localization, factory_floor=factory)
    
    # Restore original config
    cfg.NUM_AGVS = original_num_agvs
    
    print("  + Factory environment initialized")
    print("  + AGV fleet initialized")
    print("  + ISAC system initialized")
    print("  + Localization system initialized")
    print("  + Decision making system initialized")
    print()
    
    return {
        'factory': factory,
        'agv_fleet': agv_fleet,
        'isac': isac,
        'localization': localization,
        'decision_maker': decision_maker
    }


if __name__ == "__main__":
    main()


