"""
Startup Configuration Dialog for 6G ISAC Simulation

Simple dialog to configure simulation parameters before starting.
Playback controls (pause/resume/stop) are in the main visualization window.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons, TextBox
from typing import Dict, Optional, Callable
import config as cfg


class ControlPanel:
    """
    Startup configuration dialog for simulation parameters.
    """
    
    def __init__(self):
        # Configuration state
        self.config = {
            'frequency': 140.0,      # GHz
            'bandwidth': 2.0,         # GHz
            'num_agvs': 5,
            'duration': 30.0,         # seconds
            'scenario': 'normal'      # normal, collision, deadlock, threeway, crossing
        }
        
        # Callbacks
        self.on_start: Optional[Callable] = None
        
        # Create compact figure for configuration only
        self.fig = plt.figure(figsize=(6, 8))
        self.fig.canvas.manager.set_window_title('6G ISAC - Configuration')
        
        self._create_control_panel()
        
        # Status bar at bottom
        self.status_text = self.fig.text(0.5, 0.02, 'Configure parameters and press START',
                                         fontsize=9, color='#666666', ha='center')
    
    def _create_control_panel(self):
        """Create the configuration panel widgets."""
        
        # Title
        self.fig.text(0.5, 0.95, '6G ISAC Simulation', fontsize=16, fontweight='bold',
                     ha='center', color='#2C3E50')
        self.fig.text(0.5, 0.91, 'Startup Configuration', fontsize=11, ha='center', color='#7F8C8D')
        
        # Section: Radio Parameters
        self.fig.text(0.08, 0.85, '[RF] Radio Parameters', fontsize=11, fontweight='bold', color='#2980B9')
        
        # Frequency input
        self.fig.text(0.08, 0.80, 'Frequency (GHz):', fontsize=10)
        ax_freq = self.fig.add_axes([0.55, 0.79, 0.35, 0.035])
        self.freq_box = TextBox(ax_freq, '', initial=str(self.config['frequency']))
        self.freq_box.on_submit(self._on_frequency_change)
        
        # Bandwidth input
        self.fig.text(0.08, 0.74, 'Bandwidth (GHz):', fontsize=10)
        ax_bw = self.fig.add_axes([0.55, 0.73, 0.35, 0.035])
        self.bw_box = TextBox(ax_bw, '', initial=str(self.config['bandwidth']))
        self.bw_box.on_submit(self._on_bandwidth_change)
        
        # Section: Environment Settings
        self.fig.text(0.08, 0.66, '[ENV] Environment', fontsize=11, fontweight='bold', color='#27AE60')
        
        # Number of AGVs slider
        self.fig.text(0.08, 0.60, 'Number of AGVs:', fontsize=10)
        ax_agv = self.fig.add_axes([0.08, 0.55, 0.75, 0.035])
        self.agv_slider = Slider(ax_agv, '', 1, 10, valinit=self.config['num_agvs'],
                                 valstep=1, color='#3498DB')
        self.agv_slider.on_changed(self._on_agv_change)
        self.agv_label = self.fig.text(0.88, 0.555, f"[{self.config['num_agvs']}]", fontsize=11, fontweight='bold')
        # Step labels
        for i in range(1, 11):
            x_pos = 0.08 + (i - 1) * (0.75 / 9)
            self.fig.text(x_pos, 0.52, str(i), fontsize=7, color='#666666', ha='center')
        
        # Duration slider
        self.fig.text(0.08, 0.46, 'Duration (seconds):', fontsize=10)
        ax_dur = self.fig.add_axes([0.08, 0.41, 0.75, 0.035])
        self.dur_slider = Slider(ax_dur, '', 10, 120, valinit=self.config['duration'],
                                 valstep=10, color='#9B59B6')
        self.dur_slider.on_changed(self._on_duration_change)
        self.dur_label = self.fig.text(0.88, 0.415, f"[{int(self.config['duration'])}s]", fontsize=11, fontweight='bold')
        # Step labels (show every other for readability)
        for i, val in enumerate([10, 30, 50, 70, 90, 110]):
            x_pos = 0.08 + (val - 10) * (0.75 / 110)
            self.fig.text(x_pos, 0.38, str(val), fontsize=7, color='#666666', ha='center')
        
        # Section: Scenario Selection
        self.fig.text(0.08, 0.32, '[MODE] Scenario', fontsize=11, fontweight='bold', color='#E67E22')
        
        ax_scenario = self.fig.add_axes([0.08, 0.12, 0.84, 0.18])
        self.scenario_radio = RadioButtons(ax_scenario, 
                                           ('Normal Simulation', 'Collision Test', 
                                            'Deadlock Test', 'Three-Way Test', 'Crossing Test'),
                                           active=0)
        self.scenario_radio.on_clicked(self._on_scenario_change)
        
        # START button (prominent)
        ax_start = self.fig.add_axes([0.25, 0.04, 0.50, 0.06])
        self.start_btn = Button(ax_start, 'START SIMULATION', color='#27AE60', hovercolor='#2ECC71')
        self.start_btn.label.set_fontsize(12)
        self.start_btn.label.set_fontweight('bold')
        self.start_btn.on_clicked(self._on_start)
    
    def _on_frequency_change(self, text):
        try:
            val = float(text)
            if 1 <= val <= 1000:
                self.config['frequency'] = val
                self.status_text.set_text(f'Frequency: {val} GHz')
        except ValueError:
            self.status_text.set_text('Invalid frequency')
        self.fig.canvas.draw_idle()
    
    def _on_bandwidth_change(self, text):
        try:
            val = float(text)
            if 0.1 <= val <= 10:
                self.config['bandwidth'] = val
                self.status_text.set_text(f'Bandwidth: {val} GHz')
        except ValueError:
            self.status_text.set_text('Invalid bandwidth')
        self.fig.canvas.draw_idle()
    
    def _on_agv_change(self, val):
        self.config['num_agvs'] = int(val)
        self.agv_label.set_text(f"[{int(val)}]")
        self.status_text.set_text(f'AGVs: {int(val)}')
        self.fig.canvas.draw_idle()
    
    def _on_duration_change(self, val):
        self.config['duration'] = val
        self.dur_label.set_text(f"[{int(val)}s]")
        self.status_text.set_text(f'Duration: {int(val)}s')
        self.fig.canvas.draw_idle()
    
    def _on_scenario_change(self, label):
        scenario_map = {
            'Normal Simulation': 'normal',
            'Collision Test': 'collision',
            'Deadlock Test': 'deadlock',
            'Three-Way Test': 'threeway',
            'Crossing Test': 'crossing'
        }
        self.config['scenario'] = scenario_map.get(label, 'normal')
        self.status_text.set_text(f'Scenario: {label}')
        self.fig.canvas.draw_idle()
    
    def _on_start(self, event):
        """Handle start button click."""
        if self.on_start:
            self.on_start(self.config)
    
    def get_config(self) -> Dict:
        """Get current configuration."""
        return self.config.copy()
    
    def show(self):
        """Display the control panel."""
        plt.show()


if __name__ == '__main__':
    """Test the control panel standalone."""
    panel = ControlPanel()
    
    def on_start(config):
        print(f"Starting with config: {config}")
        plt.close(panel.fig)
    
    panel.on_start = on_start
    panel.show()
