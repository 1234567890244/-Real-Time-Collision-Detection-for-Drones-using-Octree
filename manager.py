import math
import time
import random
import threading
import heapq
from typing import List, Tuple, Optional, Dict
from datetime import datetime
from collections import defaultdict

from models import Drone, DroneStatus, CITY_BOUNDS, SAFETY_DISTANCE, RECOVERY_DISTANCE
from point_set import TreeNode


class DroneManager:
    def __init__(self):
        self.tree = TreeNode(CITY_BOUNDS)
        self.drones: Dict[str, Drone] = {}
        self.collisions: List[Tuple[Drone, Drone, float]] = []
        self.warnings: List[Dict] = []
        self.manual_drone_id: Optional[str] = None
        
        self.stats = {
            'total_drones': 0,
            'active_collisions': 0,
            'total_warnings': 0,
            'last_update': time.time(),
            'system_start_time': time.time(),
            'updates_count': 0
        }
        
        self.lock = threading.RLock()
        
        self.running = True
        self.collision_thread = threading.Thread(target=self.collision_detection_loop)
        self.collision_thread.daemon = True
        self.collision_thread.start()
        
        self.simulation_thread = None
        self.simulation_running = False
    
    def add_drone(self, drone: Drone) -> bool:
        with self.lock:
            if drone.id in self.drones:
                return False
            
            success = self.tree.insert(drone)
            if success:
                self.drones[drone.id] = drone
                self.stats['total_drones'] += 1
                self.stats['last_update'] = time.time()
                
                if drone.is_manual:
                    self.manual_drone_id = drone.id
            
            return success
    
    def get_drone(self, drone_id: str) -> Optional[Drone]:
        with self.lock:
            return self.drones.get(drone_id)
    
    def get_nearest_neighbors(self, drone_id: str, k: int = 5, max_distance: float = 50.0) -> List[Tuple[float, Drone]]:
        with self.lock:
            if drone_id not in self.drones:
                return []
            
            drone = self.drones[drone_id]
            neighbors = self.tree.find_nearest_neighbors(drone, k, max_distance)
            
            result = []
            for dist, neighbor in neighbors:
                result.append((-dist, neighbor))  
            
            return result
    
    def update_drone(self, drone_id: str, new_position: Tuple[float, float, float],
                    new_speeds: Tuple[float, float, float] = (0, 0, 0),
                    new_status: Optional[DroneStatus] = None) -> bool:
        with self.lock:
            if drone_id not in self.drones:
                return False
            
            drone = self.drones[drone_id]
            
            if drone.is_manual:
                new_drone = Drone(
                    id=drone.id,
                    x=new_position[0],
                    y=new_position[1],
                    z=new_position[2],
                    speed_x=drone.speed_x,
                    speed_y=drone.speed_y,
                    speed_z=drone.speed_z,
                    status=DroneStatus.MANUAL,
                    target_x=drone.target_x,
                    target_y=drone.target_y,
                    target_z=drone.target_z,
                    is_manual=True
                )
            else:
                status_to_use = new_status if new_status is not None else drone.status
                
                new_drone = Drone(
                    id=drone.id,
                    x=new_position[0],
                    y=new_position[1],
                    z=new_position[2],
                    speed_x=new_speeds[0],
                    speed_y=new_speeds[1],
                    speed_z=new_speeds[2],
                    status=status_to_use,
                    target_x=drone.target_x,
                    target_y=drone.target_y,
                    target_z=drone.target_z,
                    is_manual=drone.is_manual
                )
            
            success = self.tree.update(drone_id, new_drone)
            if success:
                self.drones[drone_id] = new_drone
                self.stats['last_update'] = time.time()
                self.stats['updates_count'] += 1
            
            return success
    
    def remove_drone(self, drone_id: str) -> bool:
        with self.lock:
            if drone_id not in self.drones:
                return False
            
            if drone_id == self.manual_drone_id:
                self.manual_drone_id = None
            
            success = self.tree.remove(drone_id)
            if success:
                del self.drones[drone_id]
                self.stats['total_drones'] -= 1
                self.stats['last_update'] = time.time()
            
            return success
    
    def change_drone_status(self, drone_id: str, new_status: DroneStatus) -> bool:
        with self.lock:
            if drone_id not in self.drones:
                return False
            
            drone = self.drones[drone_id]
            if not drone.is_manual:
                drone.status = new_status
                return True
            return False
    
    def set_drone_target(self, drone_id: str, target_x: float, target_y: float, target_z: float) -> bool:
        with self.lock:
            if drone_id not in self.drones:
                return False
            
            drone = self.drones[drone_id]
            if not drone.is_manual:
                drone.set_target(target_x, target_y, target_z)
                return True
            return False
    
    def move_manual_drone(self, dx: float = 0, dy: float = 0, dz: float = 0):
        with self.lock:
            if self.manual_drone_id and self.manual_drone_id in self.drones:
                drone = self.drones[self.manual_drone_id]
                
                new_x = drone.x + dx
                new_y = drone.y + dy
                new_z = drone.z + dz
                
                new_x = max(CITY_BOUNDS['min_x'], min(CITY_BOUNDS['max_x'], new_x))
                new_y = max(CITY_BOUNDS['min_y'], min(CITY_BOUNDS['max_y'], new_y))
                new_z = max(CITY_BOUNDS['min_z'], min(CITY_BOUNDS['max_z'], new_z))
                
                self.update_drone(self.manual_drone_id, (new_x, new_y, new_z))
                return True
            return False
    
    def find_nearest_neighbors(self, drone_id: str, k: int = 5, max_distance: float = 50.0) -> List[Tuple[Drone, float]]:
        with self.lock:
            if drone_id not in self.drones:
                return []
            
            drone = self.drones[drone_id]
            neighbors = self.tree.find_nearest_neighbors(drone, k, max_distance)
            
            result = []
            for dist, neighbor in neighbors:
                result.append((neighbor, -dist))
            
            result.sort(key=lambda x: x[1])
            return result
    
    def collision_detection_loop(self):
        while self.running:
            time.sleep(0.05)
            
            with self.lock:
                self.collisions = self.tree.find_collision_pairs(SAFETY_DISTANCE)
                self.stats['active_collisions'] = len(self.collisions)
                
                for drone1, drone2, distance in self.collisions:
                    warning = {
                        'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                        'drone1_id': drone1.id,
                        'drone2_id': drone2.id,
                        'distance': distance,
                        'position1': drone1.get_position(),
                        'position2': drone2.get_position(),
                        'status1': drone1.status.value,
                        'status2': drone2.status.value
                    }
                    self.warnings.append(warning)
                    self.stats['total_warnings'] += 1
                    
                    if drone1.is_manual or drone2.is_manual:
                        warning['manual_collision'] = True
                
                if len(self.warnings) > 100:
                    self.warnings = self.warnings[-100:]
    
    def start_simulation(self):
        if self.simulation_running:
            return
        
        self.simulation_running = True
        
        def simulate():
            while self.simulation_running and self.running:
                try:
                    drones = self.get_all_drones()
                    
                    for drone in drones:
                        if drone.is_manual:
                            continue
                        
                        dt = 0.05
                        
                        if drone.target_x is None:
                            target_x = random.uniform(CITY_BOUNDS['min_x'] + 50, CITY_BOUNDS['max_x'] - 50)
                            target_y = random.uniform(CITY_BOUNDS['min_y'] + 50, CITY_BOUNDS['max_y'] - 50)
                            target_z = random.uniform(CITY_BOUNDS['min_z'] + 50, CITY_BOUNDS['max_z'] - 50)
                            drone.set_target(target_x, target_y, target_z)
                        
                        if drone.target_x is None:
                            continue
                            
                        dx = drone.target_x - drone.x
                        dy = drone.target_y - drone.y
                        dz = drone.target_z - drone.z
                        
                        dist_to_target = math.sqrt(dx*dx + dy*dy + dz*dz)
                        
                        neighbors = self.find_nearest_neighbors(drone.id, k=1, max_distance=100.0)
                        min_distance_to_neighbor = neighbors[0][1] if neighbors else float('inf')
                        
                        new_status = drone.status
                        
                        if min_distance_to_neighbor < SAFETY_DISTANCE:
                            new_status = DroneStatus.EMERGENCY
                        elif min_distance_to_neighbor > RECOVERY_DISTANCE and drone.status == DroneStatus.EMERGENCY:
                            if dist_to_target < 20.0:
                                new_status = DroneStatus.IDLE
                            else:
                                new_status = DroneStatus.ACTIVE
                        elif dist_to_target < 20.0 and drone.status != DroneStatus.EMERGENCY:
                            new_status = DroneStatus.IDLE
                        elif drone.status == DroneStatus.IDLE and dist_to_target >= 20.0:
                            new_status = DroneStatus.ACTIVE
                        
                        if new_status == DroneStatus.IDLE and min_distance_to_neighbor >= SAFETY_DISTANCE:
                            if new_status != drone.status:
                                self.update_drone(
                                    drone.id,
                                    (drone.x, drone.y, drone.z),
                                    (0, 0, 0),
                                    new_status
                                )
                            continue
                        
                        if dist_to_target < 30.0 and new_status != DroneStatus.EMERGENCY:
                            target_x = random.uniform(CITY_BOUNDS['min_x'] + 50, CITY_BOUNDS['max_x'] - 50)
                            target_y = random.uniform(CITY_BOUNDS['min_y'] + 50, CITY_BOUNDS['max_y'] - 50)
                            target_z = random.uniform(CITY_BOUNDS['min_z'] + 50, CITY_BOUNDS['max_z'] - 50)
                            drone.set_target(target_x, target_y, target_z)
                            
                            dx = drone.target_x - drone.x
                            dy = drone.target_y - drone.y
                            dz = drone.target_z - drone.z
                            dist_to_target = math.sqrt(dx*dx + dy*dy + dz*dz)
                        
                        if dist_to_target > 0:
                            dx /= dist_to_target
                            dy /= dist_to_target
                            dz /= dist_to_target
                        
                        base_speed = 5.0
                        
                        neighbors = self.find_nearest_neighbors(drone.id, k=5, max_distance=100.0)
                        avoidance_vector = [0.0, 0.0, 0.0]
                        
                        for neighbor, distance in neighbors:
                            if distance < 60.0:
                                avoid_dx = drone.x - neighbor.x
                                avoid_dy = drone.y - neighbor.y
                                avoid_dz = drone.z - neighbor.z
                                
                                avoid_dist = math.sqrt(avoid_dx*avoid_dx + avoid_dy*avoid_dy + avoid_dz*avoid_dz)
                                if avoid_dist > 0:
                                    force_weight = 2.0 if not neighbor.is_manual else 4.0
                                    force = force_weight / max(0.1, distance)
                                    avoidance_vector[0] += force * avoid_dx / avoid_dist
                                    avoidance_vector[1] += force * avoid_dy / avoid_dist
                                    avoidance_vector[2] += force * avoid_dz / avoid_dist
                        
                        total_x = dx
                        total_y = dy
                        total_z = dz
                        
                        if math.sqrt(avoidance_vector[0]**2 + avoidance_vector[1]**2 + avoidance_vector[2]**2) > 0:
                            avoidance_weight = 5.0 if new_status == DroneStatus.EMERGENCY else 3.0
                            total_x += avoidance_vector[0] * avoidance_weight
                            total_y += avoidance_vector[1] * avoidance_weight
                            total_z += avoidance_vector[2] * avoidance_weight
                            
                            total_norm = math.sqrt(total_x**2 + total_y**2 + total_z**2)
                            if total_norm > 0:
                                total_x /= total_norm
                                total_y /= total_norm
                                total_z /= total_norm
                        
                        speed_multiplier = 1.5 if new_status == DroneStatus.EMERGENCY else 1.0
                        speed_x = total_x * base_speed * speed_multiplier
                        speed_y = total_y * base_speed * speed_multiplier
                        speed_z = total_z * (base_speed * speed_multiplier / 5)
                        
                        new_position = (
                            drone.x + speed_x * dt,
                            drone.y + speed_y * dt,
                            drone.z + speed_z * dt
                        )
                        
                        new_x, new_y, new_z = new_position
                        if new_x < CITY_BOUNDS['min_x']:
                            new_x = CITY_BOUNDS['min_x']
                            speed_x = abs(speed_x)
                        elif new_x > CITY_BOUNDS['max_x']:
                            new_x = CITY_BOUNDS['max_x']
                            speed_x = -abs(speed_x)
                        
                        if new_y < CITY_BOUNDS['min_y']:
                            new_y = CITY_BOUNDS['min_y']
                            speed_y = abs(speed_y)
                        elif new_y > CITY_BOUNDS['max_y']:
                            new_y = CITY_BOUNDS['max_y']
                            speed_y = -abs(speed_y)
                        
                        if new_z < CITY_BOUNDS['min_z']:
                            new_z = CITY_BOUNDS['min_z']
                            speed_z = abs(speed_z)
                        elif new_z > CITY_BOUNDS['max_z']:
                            new_z = CITY_BOUNDS['max_z']
                            speed_z = -abs(speed_z)
                        
                        new_position = (new_x, new_y, new_z)
                        
                        self.update_drone(
                            drone.id,
                            new_position,
                            (speed_x, speed_y, speed_z),
                            new_status
                        )
                    
                    time.sleep(0.05)
                    
                except Exception as e:
                    print(f"Simulation error: {e}")
        
        self.simulation_thread = threading.Thread(target=simulate, daemon=True)
        self.simulation_thread.start()
    
    def stop_simulation(self):
        self.simulation_running = False
        if self.simulation_thread:
            self.simulation_thread.join(timeout=2.0)
    
    def get_all_drones(self) -> List[Drone]:
        with self.lock:
            return list(self.drones.values())
    
    def get_collisions(self) -> List[Tuple[Drone, Drone, float]]:
        with self.lock:
            return self.collisions.copy()
    
    def get_stats(self) -> Dict:
        with self.lock:
            stats = self.stats.copy()
            stats['uptime'] = time.time() - stats['system_start_time']
            if stats['uptime'] > 0:
                stats['update_rate'] = stats['updates_count'] / stats['uptime']
            else:
                stats['update_rate'] = 0
            
            if self.manual_drone_id and self.manual_drone_id in self.drones:
                manual_drone = self.drones[self.manual_drone_id]
                stats['manual_drone_position'] = manual_drone.get_position()
                stats['manual_drone_exists'] = True
            else:
                stats['manual_drone_exists'] = False
            
            return stats
    
    def stop(self):
        self.stop_simulation()
        self.running = False
        if self.collision_thread.is_alive():
            self.collision_thread.join(timeout=2.0)