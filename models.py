import math
import time
from typing import List, Tuple, Optional, Dict, Set, Any
from dataclasses import dataclass, field
from datetime import datetime
from collections import defaultdict
from enum import Enum


class DroneStatus(Enum):
    ACTIVE = "active"
    IDLE = "idle"
    EMERGENCY = "emergency"
    MANUAL = "manual"


@dataclass
class Drone:
    id: str
    x: float
    y: float
    z: float
    speed_x: float = 0.0
    speed_y: float = 0.0
    speed_z: float = 0.0
    status: DroneStatus = DroneStatus.ACTIVE
    last_update: float = field(default_factory=time.time)
    
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    target_z: Optional[float] = None
    
    is_manual: bool = False
    
    def get_position(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)
    
    def get_target_position(self) -> Tuple[float, float, float]:
        if self.target_x is not None:
            return (self.target_x, self.target_y, self.target_z)
        return self.get_position()
    
    def distance_to(self, other: 'Drone') -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def distance_to_coords(self, x: float, y: float, z: float) -> float:
        dx = self.x - x
        dy = self.y - y
        dz = self.z - z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def move(self, dt: float):
        if not self.is_manual:
            self.x += self.speed_x * dt
            self.y += self.speed_y * dt
            self.z += self.speed_z * dt
        self.last_update = time.time()
    
    def set_target(self, x: float, y: float, z: float):
        if not self.is_manual:
            self.target_x = x
            self.target_y = y
            self.target_z = z
    
    def calculate_heading_to_target(self) -> Tuple[float, float, float]:
        if self.target_x is None or self.is_manual:
            return (0, 0, 0)
        
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.target_z - self.z
        
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < 1.0:
            return (0, 0, 0)
        
        return (dx/dist, dy/dist, dz/dist)


SAFETY_DISTANCE = 20.0
WARNING_DISTANCE = 40.0
RECOVERY_DISTANCE = 30.0
TREE_CAPACITY = 8

CITY_BOUNDS = {
    'min_x': 0.0,
    'max_x': 500.0,
    'min_y': 0.0,
    'max_y': 500.0,
    'min_z': 50.0,
    'max_z': 550.0
}

STATUS_COLORS = {
    DroneStatus.ACTIVE: 'green',
    DroneStatus.IDLE: 'blue',
    DroneStatus.EMERGENCY: 'red',
    DroneStatus.MANUAL: 'purple'
}

STATUS_LABELS = {
    DroneStatus.ACTIVE: 'Active',
    DroneStatus.IDLE: 'Idle',
    DroneStatus.EMERGENCY: 'Emergency',
    DroneStatus.MANUAL: 'Manual'
}