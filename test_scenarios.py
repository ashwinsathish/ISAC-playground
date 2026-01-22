"""
Test Scenarios for 6G ISAC System
Provides controlled test scenarios for debugging and demonstration.
"""

import numpy as np
from typing import Dict, List, Tuple
import config as cfg
from factory_environment import FactoryFloor
from agv import AGV, AGVFleet, AGVState
from localization import LocalizationSystem
from decision_making import DecisionMaker
from isac_system import ISACSystem


# =============================================================================
# SCENARIO DEFINITIONS
# =============================================================================

SCENARIOS = {
    'collision': {
        'description': 'Two AGVs on direct collision course',
        'num_agvs': 2,
        'initial_positions': [
            np.array([10.0, 15.0]),  # AGV 0: left side
            np.array([40.0, 15.0]),  # AGV 1: right side
        ],
        'initial_headings': [0, np.pi],  # Facing each other
        'target_waypoints': [
            np.array([40.0, 15.0]),  # AGV 0 goes right
            np.array([10.0, 15.0]),  # AGV 1 goes left
        ],
    },
    'deadlock': {
        'description': 'Two AGVs that will reach deadlock',
        'num_agvs': 2,
        'initial_positions': [
            np.array([22.0, 15.0]),  # AGV 0: slightly left of center
            np.array([28.0, 15.0]),  # AGV 1: slightly right of center
        ],
        'initial_headings': [0, np.pi],  # Facing each other
        'target_waypoints': [
            np.array([35.0, 15.0]),  # Both want to cross
            np.array([15.0, 15.0]),
        ],
    },
    'threeway': {
        'description': 'Three AGVs converging at center (triangle)',
        'num_agvs': 3,
        'initial_positions': [
            np.array([25.0, 5.0]),   # Bottom
            np.array([15.0, 25.0]),  # Top-left
            np.array([35.0, 25.0]),  # Top-right
        ],
        'initial_headings': [np.pi/2, -np.pi/4, -3*np.pi/4],  # All pointing to center
        'target_waypoints': [
            np.array([25.0, 20.0]),  # All converge at center
            np.array([25.0, 15.0]),
            np.array([25.0, 15.0]),
        ],
    },
    'crossing': {
        'description': 'Two AGVs crossing paths at right angles',
        'num_agvs': 2,
        'initial_positions': [
            np.array([15.0, 15.0]),  # West
            np.array([25.0, 5.0]),   # South
        ],
        'initial_headings': [0, np.pi/2],  # Perpendicular paths
        'target_waypoints': [
            np.array([35.0, 15.0]),  # Goes east
            np.array([25.0, 25.0]),  # Goes north
        ],
    },
}


# =============================================================================
# SCENARIO CREATION
# =============================================================================

class ScenarioAGVFleet(AGVFleet):
    """AGV Fleet that can be initialized with custom positions."""
    
    def __init__(self, factory_floor, scenario_config: Dict):
        self.factory_floor = factory_floor
        self.agvs: List[AGV] = []
        
        positions = scenario_config['initial_positions']
        headings = scenario_config['initial_headings']
        waypoints = scenario_config['target_waypoints']
        
        for i, (pos, heading, target) in enumerate(zip(positions, headings, waypoints)):
            agv = AGV(agv_id=i, initial_position=pos)
            agv.heading = heading
            agv.target_waypoint = target.copy()
            agv.state = AGVState.MOVING
            self.agvs.append(agv)


def create_scenario_simulation(scenario_name: str) -> Dict:
    """
    Create simulation with a specific test scenario.
    
    Args:
        scenario_name: Name of the scenario (collision, deadlock, threeway, crossing)
        
    Returns:
        Dict of simulation components
    """
    if scenario_name not in SCENARIOS:
        available = ', '.join(SCENARIOS.keys())
        raise ValueError(f"Unknown scenario '{scenario_name}'. Available: {available}")
    
    scenario = SCENARIOS[scenario_name]
    num_agvs = scenario['num_agvs']
    
    print("=" * 60)
    print(f"6G ISAC System - Test Scenario: {scenario_name.upper()}")
    print("=" * 60)
    print(f"\n{scenario['description']}")
    print(f"\nScenario Parameters:")
    print(f"  • Number of AGVs: {num_agvs}")
    for i, pos in enumerate(scenario['initial_positions']):
        target = scenario['target_waypoints'][i]
        print(f"  • AGV {i}: Start ({pos[0]:.0f}, {pos[1]:.0f}) → Target ({target[0]:.0f}, {target[1]:.0f})")
    print()
    
    # Initialize components
    print("Initializing scenario...")
    factory = FactoryFloor()
    agv_fleet = ScenarioAGVFleet(factory, scenario)
    isac = ISACSystem()
    localization = LocalizationSystem(num_agvs)
    
    # Initialize localization with actual positions
    for agv in agv_fleet.agvs:
        localization.trackers[agv.id].state[:2] = agv.position
        
    decision_maker = DecisionMaker(localization, factory_floor=factory)
    
    print("  ✓ Scenario loaded")
    print()
    
    return {
        'factory': factory,
        'agv_fleet': agv_fleet,
        'isac': isac,
        'localization': localization,
        'decision_maker': decision_maker,
        'scenario_name': scenario_name,
        'scenario': scenario
    }


def list_scenarios():
    """Print available test scenarios."""
    print("\nAvailable Test Scenarios:")
    print("-" * 40)
    for name, config in SCENARIOS.items():
        print(f"  {name:12} - {config['description']}")
    print()
