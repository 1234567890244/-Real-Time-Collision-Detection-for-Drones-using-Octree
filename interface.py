import tkinter as tk
from tkinter import ttk, scrolledtext
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import random
import math
from collections import defaultdict
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from manager import DroneManager
    from models import DroneStatus, Drone

from models import Drone, DroneStatus, STATUS_COLORS, STATUS_LABELS, CITY_BOUNDS, SAFETY_DISTANCE


class DroneVisualizer:
    def __init__(self, manager: 'DroneManager'):
        self.manager = manager
        self.root = tk.Tk()
        self.root.title("Drone Collision Warning System")
        self.root.geometry("1200x800")
        
        self.style = ttk.Style()
        self.style.theme_use('clam')
        
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.control_frame = ttk.LabelFrame(self.main_frame, text="Control Panel", width=200)
        self.control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        self.viz_frame = ttk.LabelFrame(self.main_frame, text="Airspace Visualization")
        self.viz_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.trajectory_history = defaultdict(list)
        
        self.init_control_panel()
        self.init_visualization()
        self.init_status_panel()
        
        self.update_interval = 100
        self.root.after(self.update_interval, self.update_display)
    
    def generate_drones(self):
        try:
            num_drones = int(self.drone_count_var.get())
        except ValueError:
            num_drones = 100
        
        self.clear_drones()

        positions = []
        attempts = 0
        max_attempts = num_drones * 10
        
        for i in range(num_drones):
            placed = False
            while not placed and attempts < max_attempts:
                x = random.uniform(CITY_BOUNDS['min_x'] + 50, CITY_BOUNDS['max_x'] - 50)
                y = random.uniform(CITY_BOUNDS['min_y'] + 50, CITY_BOUNDS['max_y'] - 50)
                z = random.uniform(CITY_BOUNDS['min_z'] + 50, CITY_BOUNDS['max_z'] - 50)
                
                too_close = False
                for (existing_x, existing_y, existing_z) in positions:
                    dx = x - existing_x
                    dy = y - existing_y
                    dz = z - existing_z
                    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                    
                    if distance < 50.0:
                        too_close = True
                        break
                
                if not too_close:
                    positions.append((x, y, z))
                    placed = True
                else:
                    attempts += 1
            
            if not placed:
                x = random.uniform(CITY_BOUNDS['min_x'] + 10, CITY_BOUNDS['max_x'] - 10)
                y = random.uniform(CITY_BOUNDS['min_y'] + 10, CITY_BOUNDS['max_y'] - 10)
                z = random.uniform(CITY_BOUNDS['min_z'] + 10, CITY_BOUNDS['max_z'] - 10)
                positions.append((x, y, z))
        
        for i in range(num_drones):
            x, y, z = positions[i]
            
            initial_status = DroneStatus.ACTIVE
            
            for j in range(num_drones):
                if i != j:
                    other_x, other_y, other_z = positions[j]
                    dx = x - other_x
                    dy = y - other_y
                    dz = z - other_z
                    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                    
                    if distance < SAFETY_DISTANCE:
                        initial_status = DroneStatus.EMERGENCY
                        break
            
            base_speed = 5.0
            angle = random.uniform(0, 2 * math.pi)
            speed_x = math.cos(angle) * base_speed
            speed_y = math.sin(angle) * base_speed
            speed_z = random.uniform(-1, 1)
            
            while True:
                target_x = x + random.uniform(-100, 100)
                target_y = y + random.uniform(-100, 100)
                target_z = z + random.uniform(-30, 30)
                
                target_x = max(CITY_BOUNDS['min_x'] + 10, min(CITY_BOUNDS['max_x'] - 10, target_x))
                target_y = max(CITY_BOUNDS['min_y'] + 10, min(CITY_BOUNDS['max_y'] - 10, target_y))
                target_z = max(CITY_BOUNDS['min_z'] + 10, min(CITY_BOUNDS['max_z'] - 10, target_z))
                
                dx = target_x - x
                dy = target_y - y
                dz = target_z - z
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if dist > 50.0:
                    break
            
            drone = Drone(
                id=f"drone-{i:04d}",
                x=x,
                y=y,
                z=z,
                speed_x=speed_x,
                speed_y=speed_y,
                speed_z=speed_z,
                status=initial_status,
                is_manual=False
            )

            drone.set_target(target_x, target_y, target_z)
            
            if self.manager.add_drone(drone):
                status_str = "EMERGENCY" if initial_status == DroneStatus.EMERGENCY else "ACTIVE"
            else:
                print("Error:", drone.id)
    
    def clear_drones(self):
        drones = self.manager.get_all_drones()
        for drone in drones:
            self.manager.remove_drone(drone.id)
        
        self.trajectory_history.clear()

    def start_simulation(self):
        self.manager.start_simulation()
        self.sim_start_btn.config(state=tk.DISABLED)
        self.sim_stop_btn.config(state=tk.NORMAL)
    
    def stop_simulation(self):
        self.manager.stop_simulation()
        self.sim_start_btn.config(state=tk.NORMAL)
        self.sim_stop_btn.config(state=tk.DISABLED)
    
    def setup_plot(self):
        self.ax.clear()
        
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        
        self.ax.set_xlim(CITY_BOUNDS['min_x'], CITY_BOUNDS['max_x'])
        self.ax.set_ylim(CITY_BOUNDS['min_y'], CITY_BOUNDS['max_y'])
        self.ax.set_zlim(CITY_BOUNDS['min_z'], CITY_BOUNDS['max_z'])
        
        legend_elements = [
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=8, label='Active'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=8, label='Idle'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=8, label='Emergency'),
            plt.Line2D([0], [0], color='red', linewidth=2, label='Collision Line')
        ]
        self.ax.legend(handles=legend_elements, loc='upper right')
    
    def init_control_panel(self):
        sim_frame = ttk.LabelFrame(self.control_frame, text="Simulation Control")
        sim_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.sim_start_btn = ttk.Button(sim_frame, text="Start Simulation", command=self.start_simulation)
        self.sim_start_btn.pack(pady=5, fill=tk.X)
        
        self.sim_stop_btn = ttk.Button(sim_frame, text="Stop Simulation", command=self.stop_simulation, state=tk.DISABLED)
        self.sim_stop_btn.pack(pady=5, fill=tk.X)
        
        drone_frame = ttk.LabelFrame(self.control_frame, text="Drone Management")
        drone_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(drone_frame, text="Number of Drones:").pack(anchor=tk.W, padx=5, pady=5)
        
        self.drone_count_var = tk.StringVar(value="100")
        count_frame = ttk.Frame(drone_frame)
        count_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Spinbox(count_frame, from_=1, to=100, textvariable=self.drone_count_var, width=15).pack(side=tk.LEFT)
        
        ttk.Button(drone_frame, text="Generate Drones", command=self.generate_drones).pack(pady=5, fill=tk.X)
        ttk.Button(drone_frame, text="Clear All Drones", command=self.clear_drones).pack(pady=5, fill=tk.X)
    
    def init_visualization(self):
        self.fig = Figure(figsize=(10, 8), dpi=100)
        
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.viz_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.drone_scatter = None
        self.collision_lines = []
        self.drone_labels = []
        
        self.setup_plot()
    
    def init_status_panel(self):
        status_frame = ttk.LabelFrame(self.main_frame, text="System Status & Warnings")
        status_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)
        
        ttk.Label(status_frame, text="Collision Warnings:", font=('Arial', 10, 'bold')).pack(anchor=tk.W, padx=5, pady=2)
        
        self.warning_text = scrolledtext.ScrolledText(status_frame, height=4, width=100)
        self.warning_text.pack(fill=tk.X, padx=5, pady=5)
        self.warning_text.config(state=tk.DISABLED)
    
    def update_display(self):
        try:
            drones = self.manager.get_all_drones()
            collisions = self.manager.get_collisions()
            stats = self.manager.get_stats()

            self.ax.clear()
            self.setup_plot()
            
            if not drones:
                self.canvas.draw()
                if self.root.winfo_exists():
                    self.root.after(self.update_interval, self.update_display)
                return
            
            xs = []
            ys = []
            zs = []
            colors = []
            sizes = []
            
            status_counts = defaultdict(int)
            
            for drone in drones:
                xs.append(drone.x)
                ys.append(drone.y)
                zs.append(drone.z)
                colors.append(STATUS_COLORS[drone.status])
                
                base_size = 40
                if drone.status == DroneStatus.EMERGENCY:
                    size = base_size * 2.0
                elif drone.status == DroneStatus.ACTIVE:
                    size = base_size * 1.5
                else:
                    size = base_size
                
                sizes.append(size)
                status_counts[drone.status] += 1

            self.drone_scatter = self.ax.scatter(xs, ys, zs, s=sizes, c=colors,
                                               alpha=0.8, edgecolors='black', linewidths=1)
            
            if collisions:
                for drone1, drone2, distance in collisions:
                    line, = self.ax.plot([drone1.x, drone2.x],
                                       [drone1.y, drone2.y],
                                       [drone1.z, drone2.z],
                                       'r-', linewidth=2, alpha=0.7)
                    
                    self.collision_lines.append(line)
                    
                    mid_x = (drone1.x + drone2.x) / 2
                    mid_y = (drone1.y + drone2.y) / 2
                    mid_z = (drone1.z + drone2.z) / 2
                    
                    self.ax.text(mid_x, mid_y, mid_z, f'{distance:.1f}m',
                               fontsize=8, color='red', ha='center', va='center',
                               bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
            
            self.warning_text.config(state=tk.NORMAL)
            self.warning_text.delete(1.0, tk.END)
            
            if collisions:
                self.warning_text.insert(tk.END, "CURRENT COLLISION WARNINGS (<20m):\n")
                for i, (drone1, drone2, distance) in enumerate(collisions[:5]):
                    self.warning_text.insert(tk.END, f"{i+1}. {drone1.id} ↔ {drone2.id}: {distance:.1f}m\n")
            else:
                self.warning_text.insert(tk.END, "No collision warnings (<20m)\n")
            
            self.warning_text.config(state=tk.DISABLED)
            
            self.canvas.draw()
            
            self.collision_lines.clear()
            
        except Exception as e:
            print(e)
        
        if self.root.winfo_exists():
            self.root.after(self.update_interval, self.update_display)
    
    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()
    
    def on_closing(self):
        self.cleanup()
        self.root.destroy()
    
    def cleanup(self):
        self.manager.stop()