"""
Visualization System
Real-time visualization of the ISAC factory simulation using matplotlib.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from typing import Dict, List, Optional, Tuple
import config as cfg
from decision_making import SafetyZone


class FactoryVisualizer:
    """
    Real-time visualization of the factory floor and ISAC system.
    """
    
    def __init__(self, factory_floor, agv_fleet, isac_system, 
                 localization_system, decision_maker):
        self.factory = factory_floor
        self.agv_fleet = agv_fleet
        self.isac = isac_system
        self.localization = localization_system
        self.decision_maker = decision_maker
        
        # Create figure with subplots
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('6G ISAC System - Industrial Factory Floor', 
                         fontsize=14, fontweight='bold', y=0.98)
        
        # Display simulation parameters below title
        num_agvs = len(agv_fleet.agvs)
        params_text = f"Freq: {cfg.CARRIER_FREQUENCY/1e9:.0f} GHz | BW: {cfg.BANDWIDTH/1e9:.1f} GHz | AGVs: {num_agvs} | Factory: {cfg.FACTORY_LENGTH}m x {cfg.FACTORY_WIDTH}m"
        self.fig.text(0.5, 0.95, params_text, ha='center', fontsize=10, color='#555555')
        
        # Main factory view
        self.ax_main = self.fig.add_subplot(2, 2, (1, 3))
        self.ax_main.set_xlim(-2, cfg.FACTORY_LENGTH + 2)
        self.ax_main.set_ylim(-2, cfg.FACTORY_WIDTH + 2)
        self.ax_main.set_aspect('equal')
        self.ax_main.set_xlabel('X position (m)')
        self.ax_main.set_ylabel('Y position (m)')
        self.ax_main.set_title('Factory Floor - Top View')
        self.ax_main.grid(True, alpha=0.3)
        
        # Performance metrics
        self.ax_metrics = self.fig.add_subplot(2, 2, 2)
        self.ax_metrics.set_title('Localization Performance')
        
        # System status
        self.ax_status = self.fig.add_subplot(2, 2, 4)
        self.ax_status.set_title('System Status')
        self.ax_status.axis('off')
        
        # Draw static elements
        self._draw_factory()
        
        # Initialize dynamic elements
        self.agv_patches = {}
        self.agv_trails = {}
        self.agv_estimates = {}
        self.agv_predictions = {}
        self.agv_waypoints = {}  # Waypoint markers
        self.beam_lines = []
        self.safety_zones = {}
        
        self._init_agv_elements()
        
        # Error history for plotting
        self.error_history = []
        self.time_history = []
        self.current_time = 0
        
        plt.tight_layout()
        
    def _draw_factory(self):
        """Draw static factory elements."""
        # Factory floor boundary
        factory_rect = patches.Rectangle(
            (0, 0), cfg.FACTORY_LENGTH, cfg.FACTORY_WIDTH,
            linewidth=2, edgecolor='black', facecolor='#f0f0f0'
        )
        self.ax_main.add_patch(factory_rect)
        
        # Storage racks
        for i, rack in enumerate(cfg.STORAGE_RACKS):
            rack_patch = patches.Rectangle(
                (rack[0], rack[1]), rack[2] - rack[0], rack[3] - rack[1],
                linewidth=1, edgecolor='#444444', facecolor='#8B4513',
                alpha=0.7
            )
            self.ax_main.add_patch(rack_patch)
            self.ax_main.text(
                (rack[0] + rack[2]) / 2, (rack[1] + rack[3]) / 2,
                f'Rack {i+1}', ha='center', va='center', fontsize=8,
                color='white', fontweight='bold'
            )
        
        # Base station
        bs_marker = self.ax_main.plot(
            cfg.BS_POSITION[0], cfg.BS_POSITION[1], 
            marker='v', markersize=15, color='red',
            markeredgecolor='darkred', markeredgewidth=2,
            label='Base Station'
        )[0]
        
        # Legend
        self.ax_main.legend(loc='upper right')
        
    def _init_agv_elements(self):
        """Initialize AGV visual elements."""
        num_agvs = len(self.agv_fleet.agvs)
        colors = plt.cm.tab10(np.linspace(0, 1, max(num_agvs, 1)))
        
        for i, agv in enumerate(self.agv_fleet.agvs):
            color = colors[i]
            
            # AGV body (rectangle)
            agv_patch = patches.Rectangle(
                (agv.position[0] - agv.length/2, agv.position[1] - agv.width/2),
                agv.length, agv.width,
                angle=np.degrees(agv.heading),
                linewidth=1, edgecolor='black', facecolor=color,
                alpha=0.8
            )
            self.ax_main.add_patch(agv_patch)
            self.agv_patches[i] = agv_patch
            
            # Trail
            trail, = self.ax_main.plot([], [], '-', color=color, 
                                       alpha=0.5, linewidth=1)
            self.agv_trails[i] = trail
            
            # Estimated position (circle)
            estimate, = self.ax_main.plot([], [], 'o', color=color,
                                         markersize=8, alpha=0.6,
                                         markeredgecolor='white',
                                         markeredgewidth=1)
            self.agv_estimates[i] = estimate
            
            # Predicted trajectory
            prediction, = self.ax_main.plot([], [], '--', color=color,
                                           alpha=0.4, linewidth=1.5)
            self.agv_predictions[i] = prediction
            
            # Safety zone circle
            safety_circle = patches.Circle(
                (agv.position[0], agv.position[1]), 
                cfg.SAFETY_ZONE_YELLOW,
                linewidth=1, edgecolor='yellow', facecolor='none',
                alpha=0.3
            )
            self.ax_main.add_patch(safety_circle)
            self.safety_zones[i] = safety_circle
            
            # Waypoint marker (star shape)
            waypoint_marker, = self.ax_main.plot([], [], '*', color=color,
                                                  markersize=12, alpha=0.8,
                                                  markeredgecolor='black',
                                                  markeredgewidth=0.5)
            self.agv_waypoints[i] = waypoint_marker
    
    def update(self, frame_info: dict):
        """
        Update visualization with new frame data.
        
        Args:
            frame_info: Dict containing current simulation state
        """
        self.current_time = frame_info.get('time', self.current_time)
        
        # Update AGV positions
        for i, agv in enumerate(self.agv_fleet.agvs):
            # Update AGV rectangle
            patch = self.agv_patches[i]
            transform = (plt.matplotlib.transforms.Affine2D()
                        .rotate_around(agv.position[0], agv.position[1], 
                                      agv.heading)
                        + self.ax_main.transData)
            patch.set_xy((agv.position[0] - agv.length/2, 
                         agv.position[1] - agv.width/2))
            patch.set_transform(transform)
            
            # Update trail
            if agv.position_history:
                trail_data = np.array(agv.position_history)
                self.agv_trails[i].set_data(trail_data[:, 0], trail_data[:, 1])
            
            # Update waypoint marker
            if agv.target_waypoint is not None:
                self.agv_waypoints[i].set_data([agv.target_waypoint[0]], 
                                               [agv.target_waypoint[1]])
            else:
                self.agv_waypoints[i].set_data([], [])
            
            # Update safety zone
            self.safety_zones[i].center = (agv.position[0], agv.position[1])
        
        # Update estimated positions
        estimates = frame_info.get('estimates', {})
        for agv_id, est_pos in estimates.items():
            self.agv_estimates[agv_id].set_data([est_pos[0]], [est_pos[1]])
        
        # Update predicted trajectories
        predictions = frame_info.get('predictions', {})
        for agv_id, trajectory in predictions.items():
            if trajectory:
                traj_array = np.array(trajectory)
                self.agv_predictions[agv_id].set_data(
                    traj_array[:, 0], traj_array[:, 1]
                )
        
        # Update safety zone colors based on collision risks
        risks = frame_info.get('awareness', {}).get('risk_pairs', [])
        for i in self.safety_zones.keys():
            self.safety_zones[i].set_edgecolor('green')
            self.safety_zones[i].set_alpha(0.2)
        
        for id1, id2, severity in risks:
            if severity == 'RED':
                self.safety_zones[id1].set_edgecolor('red')
                self.safety_zones[id2].set_edgecolor('red')
                self.safety_zones[id1].set_alpha(0.5)
                self.safety_zones[id2].set_alpha(0.5)
            elif severity == 'YELLOW':
                if self.safety_zones[id1].get_edgecolor()[:3] != (1, 0, 0):
                    self.safety_zones[id1].set_edgecolor('orange')
                    self.safety_zones[id1].set_alpha(0.4)
                if self.safety_zones[id2].get_edgecolor()[:3] != (1, 0, 0):
                    self.safety_zones[id2].set_edgecolor('orange')
                    self.safety_zones[id2].set_alpha(0.4)
        
        # Update metrics plot
        if 'rmse' in frame_info:
            self.error_history.append(frame_info['rmse'])
            self.time_history.append(self.current_time)
            
            # Keep last 200 samples
            if len(self.error_history) > 200:
                self.error_history.pop(0)
                self.time_history.pop(0)
            
            self.ax_metrics.clear()
            self.ax_metrics.plot(self.time_history, self.error_history, 'b-', linewidth=1.5)
            self.ax_metrics.axhline(y=0.2, color='r', linestyle='--', 
                                   label='Target (20 cm)', alpha=0.7)
            self.ax_metrics.set_xlabel('Time (s)')
            self.ax_metrics.set_ylabel('RMSE (m)')
            self.ax_metrics.set_title('Localization RMSE')
            self.ax_metrics.legend(loc='upper right')
            self.ax_metrics.grid(True, alpha=0.3)
            self.ax_metrics.set_ylim(0, max(0.5, max(self.error_history) * 1.1))
        
        # Update status text
        self._update_status_text(frame_info)
        
    def _update_status_text(self, frame_info: dict):
        """Update the status panel with current metrics."""
        self.ax_status.clear()
        self.ax_status.axis('off')
        
        # Compile status information
        metrics = frame_info.get('isac_metrics', {})
        awareness = frame_info.get('awareness', {})
        decision_stats = frame_info.get('decision_stats', {})
        num_agvs = len(self.agv_fleet.agvs)
        
        # Clean modern layout with sections
        y_pos = 0.98
        line_height = 0.065
        
        # Title
        self.ax_status.text(0.5, y_pos, 'SYSTEM STATUS', transform=self.ax_status.transAxes,
                           fontsize=11, fontweight='bold', ha='center', color='#2C3E50')
        y_pos -= line_height * 1.0
        
        # Time display
        self.ax_status.text(0.5, y_pos, f'Time: {self.current_time:.2f}s', 
                           transform=self.ax_status.transAxes,
                           fontsize=10, ha='center', color='#7F8C8D', fontweight='bold')
        y_pos -= line_height * 1.2
        
        # --- SENSING Section ---
        self.ax_status.text(0.05, y_pos, 'SENSING', transform=self.ax_status.transAxes,
                           fontsize=9, fontweight='bold', color='#3498DB')
        y_pos -= line_height * 0.7
        
        tracked = metrics.get('num_agvs_tracked', num_agvs)
        detection_rate = metrics.get('detection_rate', 1.0) * 100
        snr = metrics.get('average_sensing_snr_db', 0)
        
        self.ax_status.text(0.08, y_pos, f'AGVs Tracked: {tracked}/{num_agvs}', 
                           transform=self.ax_status.transAxes, fontsize=8, color='#333333')
        y_pos -= line_height * 0.6
        self.ax_status.text(0.08, y_pos, f'Detection Rate: {detection_rate:.1f}%', 
                           transform=self.ax_status.transAxes, fontsize=8, color='#333333')
        y_pos -= line_height * 0.6
        self.ax_status.text(0.08, y_pos, f'Avg Sensing SNR: {snr:.1f} dB', 
                           transform=self.ax_status.transAxes, fontsize=8, color='#333333')
        y_pos -= line_height * 0.9
        
        # --- LOCALIZATION Section ---
        self.ax_status.text(0.05, y_pos, 'LOCALIZATION', transform=self.ax_status.transAxes,
                           fontsize=9, fontweight='bold', color='#27AE60')
        y_pos -= line_height * 0.7
        
        rmse_cm = frame_info.get('rmse', 0) * 100
        range_res = cfg.RANGE_RESOLUTION * 100
        
        rmse_color = '#27AE60' if rmse_cm < 10 else '#F39C12' if rmse_cm < 20 else '#E74C3C'
        self.ax_status.text(0.08, y_pos, f'RMSE: {rmse_cm:.2f} cm', 
                           transform=self.ax_status.transAxes, fontsize=8, color=rmse_color,
                           fontweight='bold')
        y_pos -= line_height * 0.6
        self.ax_status.text(0.08, y_pos, f'Range Resolution: {range_res:.1f} cm', 
                           transform=self.ax_status.transAxes, fontsize=8, color='#333333')
        y_pos -= line_height * 0.9
        
        # --- COMMUNICATION Section ---
        self.ax_status.text(0.05, y_pos, 'COMMUNICATION', transform=self.ax_status.transAxes,
                           fontsize=9, fontweight='bold', color='#9B59B6')
        y_pos -= line_height * 0.7
        
        throughput = metrics.get('average_comm_throughput_mbps', 0)
        total_data = metrics.get('total_data_gbits', 0)
        
        self.ax_status.text(0.08, y_pos, f'Avg Throughput: {throughput:.1f} Mbps', 
                           transform=self.ax_status.transAxes, fontsize=8, color='#333333')
        y_pos -= line_height * 0.6
        self.ax_status.text(0.08, y_pos, f'Total Data: {total_data:.4f} Gbits', 
                           transform=self.ax_status.transAxes, fontsize=8, color='#333333')
        y_pos -= line_height * 0.9
        
        # --- SITUATIONAL AWARENESS Section ---
        self.ax_status.text(0.05, y_pos, 'SITUATIONAL AWARENESS', transform=self.ax_status.transAxes,
                           fontsize=9, fontweight='bold', color='#E74C3C')
        y_pos -= line_height * 0.7
        
        risks = awareness.get('total_risks', 0)
        red_zones = awareness.get('red_zone_count', 0)
        near_miss = decision_stats.get('near_miss_events', 0)
        
        risk_color = '#E74C3C' if red_zones > 0 else '#F39C12' if risks > 0 else '#27AE60'
        self.ax_status.text(0.08, y_pos, f'Active Collision Risks: {risks}', 
                           transform=self.ax_status.transAxes, fontsize=8, color=risk_color)
        y_pos -= line_height * 0.6
        self.ax_status.text(0.08, y_pos, f'Red Zone Warnings: {red_zones}', 
                           transform=self.ax_status.transAxes, fontsize=8, 
                           color='#E74C3C' if red_zones > 0 else '#333333')
        y_pos -= line_height * 0.6
        self.ax_status.text(0.08, y_pos, f'Near-Miss Events: {near_miss}', 
                           transform=self.ax_status.transAxes, fontsize=8, color='#333333')
    
    def show(self):
        """Display the visualization window."""
        plt.show(block=False)
        plt.pause(0.01)
    
    def save_frame(self, filename: str):
        """Save current frame as image."""
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
    
    def close(self):
        """Close the visualization window."""
        plt.close(self.fig)


class SimulationRenderer:
    """
    Renders the simulation with real-time updates.
    """
    
    def __init__(self, visualizer: FactoryVisualizer, update_rate: int = None):
        self.visualizer = visualizer
        self.update_rate = update_rate or cfg.VIZ_UPDATE_RATE
        self.frame_interval = 1000 // self.update_rate  # milliseconds
        self.is_running = False
        
    def start(self, simulation_generator):
        """
        Start the visualization with data from simulation generator.
        
        Args:
            simulation_generator: Generator yielding frame data
        """
        self.is_running = True
        self.gen = simulation_generator  # Store generator reference
        
        def animate(frame):
            if not self.is_running:
                return
            
            try:
                frame_data = next(self.gen)
                self.visualizer.update(frame_data)
            except StopIteration:
                self.is_running = False
        
        self.animation = FuncAnimation(
            self.visualizer.fig, animate,
            interval=self.frame_interval,
            blit=False, cache_frame_data=False
        )
        
        plt.show()
    
    def stop(self):
        """Stop the animation."""
        self.is_running = False
        if hasattr(self, 'animation'):
            self.animation.event_source.stop()
