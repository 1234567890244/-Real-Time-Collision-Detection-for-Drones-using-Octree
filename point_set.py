import math
import heapq
from typing import List, Optional, Dict, Tuple
from models import Drone, CITY_BOUNDS, TREE_CAPACITY, SAFETY_DISTANCE


class TreeNode:
    def __init__(self, bounds: Dict[str, float], depth: int = 0, max_depth: int = 15):
        self.bounds = bounds 
        self.depth = depth   
        self.max_depth = max_depth 
        self.leaf = True     
        self.drones: List[Drone] = []  
        self.children: List[Optional['TreeNode']] = [None] * 8  
        self.total_drones = 0  
    
    def get_child_index(self, drone: Drone) -> int:
        mid_x = (self.bounds['min_x'] + self.bounds['max_x']) / 2
        mid_y = (self.bounds['min_y'] + self.bounds['max_y']) / 2
        mid_z = (self.bounds['min_z'] + self.bounds['max_z']) / 2
        
        child_id = 0
        if drone.x >= mid_x:
            child_id |= 1 
        if drone.y >= mid_y:
            child_id |= 2 
        if drone.z >= mid_z:
            child_id |= 4  
        
        return child_id  
    
    def create_child(self, child_id: int) -> 'TreeNode':
        mid_x = (self.bounds['min_x'] + self.bounds['max_x']) / 2
        mid_y = (self.bounds['min_y'] + self.bounds['max_y']) / 2
        mid_z = (self.bounds['min_z'] + self.bounds['max_z']) / 2
        
        if child_id & 1: 
            child_min_x = mid_x
            child_max_x = self.bounds['max_x']
        else:  
            child_min_x = self.bounds['min_x']
            child_max_x = mid_x
        
        if child_id & 2:  
            child_min_y = mid_y
            child_max_y = self.bounds['max_y']
        else:  
            child_min_y = self.bounds['min_y']
            child_max_y = mid_y
        
        if child_id & 4:  
            child_min_z = mid_z
            child_max_z = self.bounds['max_z']
        else:  
            child_min_z = self.bounds['min_z']
            child_max_z = mid_z
        
        return TreeNode({
            'min_x': child_min_x,
            'max_x': child_max_x,
            'min_y': child_min_y,
            'max_y': child_max_y,
            'min_z': child_min_z,
            'max_z': child_max_z
        }, self.depth + 1, self.max_depth)
    
    def insert(self, drone: Drone) -> bool:
        """
        Insert a drone into the octree using divide-and-conquer.
        Divide: Split node into 8 child nodes when capacity exceeded.
        Conquer: Recursively insert drone into appropriate child.
        Merge: Maintain counts and structure integrity.
        """
        # Drone outside bounds
        if not (self.bounds['min_x'] <= drone.x <= self.bounds['max_x'] and
                self.bounds['min_y'] <= drone.y <= self.bounds['max_y'] and
                self.bounds['min_z'] <= drone.z <= self.bounds['max_z']):
            return False
        
        self.total_drones += 1
        
        # Base case: Leaf with capacity available
        if self.leaf and len(self.drones) < TREE_CAPACITY:
            self.drones.append(drone)
            return True
        
        # Divide: Split leaf node when capacity exceeded
        if self.leaf:
            # Max depth reached
            if self.depth >= self.max_depth:
                self.drones.append(drone)
                return True
            
            for i in range(8):
                self.children[i] = self.create_child(i)
            
            # Assign existing drones to children
            drones_to_reinsert = self.drones.copy()
            self.drones.clear()
            
            for d in drones_to_reinsert:
                child_id = self.get_child_index(d)
                self.children[child_id].insert(d)  
            
            self.leaf = False 
        
        # Conquer: Recursively insert into appropriate child
        child_id = self.get_child_index(drone)
        if self.children[child_id] is None:
            self.children[child_id] = self.create_child(child_id)
        
        # Recursive call (Conquer step)
        return self.children[child_id].insert(drone)
    
    def remove(self, drone_id: str) -> bool:
        if self.leaf:
            # Base case: Remove from leaf node
            for i, d in enumerate(self.drones):
                if d.id == drone_id:
                    self.drones.pop(i)
                    self.total_drones -= 1
                    return True
            return False
        
        # Recursively search in children
        for child in self.children:
            if child and child.remove(drone_id):
                self.total_drones -= 1
                
                # Merge: Merge children if total drones below capacity
                total_child_drones = sum(c.total_drones for c in self.children if c)
                if total_child_drones <= TREE_CAPACITY:
                    self._merge_children()
                
                return True
        
        return False
    
    def _merge_children(self):
        all_drones = []
        for child in self.children:
            if child:
                all_drones.extend(child.collect_all_drones())
                child.drones.clear()
                child.children = [None] * 8
                child.leaf = True
        
        # Combine all drones into this node
        self.drones = all_drones
        self.children = [None] * 8
        self.leaf = True
    
    def update(self, drone_id: str, new_drone: Drone) -> bool:
        if self.remove(drone_id):
            return self.insert(new_drone)
        return False
    
    def collect_all_drones(self) -> List[Drone]:
        drones = self.drones.copy()
        if not self.leaf:
            # Recursively collect from children
            for child in self.children:
                if child:
                    drones.extend(child.collect_all_drones())
        return drones
    
    def find_drone(self, drone_id: str) -> Optional[Drone]:
        """Find specific drone in subtree."""
        if self.leaf:
            # Base case: Check local drones
            for d in self.drones:
                if d.id == drone_id:
                    return d
            return None
        
        # Recursive search in children
        for child in self.children:
            if child:
                result = child.find_drone(drone_id)
                if result:
                    return result
        
        return None
    
    def find_nearest_neighbors(self, drone: Drone, k: int = 5,
                              max_distance: float = 100.0) -> List[Tuple[float, Drone]]:
        return self.find_k_nearest(drone, k, max_distance)
    
    def find_k_nearest(self, drone: Drone, k: int, max_distance: float,
                       best: List[Tuple[float, Drone]] = None) -> List[Tuple[float, Drone]]:
        """
        Find k nearest neighbors using divide-and-conquer with pruning.
        Divide: Search space partitioned into subspaces.
        Conquer: Recursively search in child nodes.
        Merge: Merge results from multiple subspaces.
        """
        if best is None:
            best = []  
        
        # Base case: Search within leaf node
        if self.leaf:
            for other_drone in self.drones:
                if other_drone.id != drone.id:
                    distance = drone.distance_to(other_drone)
                    if distance <= max_distance:
                        heapq.heappush(best, (-distance, other_drone))
                        if len(best) > k:
                            heapq.heappop(best)  
            return best
        
        min_dist_to_node = self.min_distance_to_bounds(drone)
        
        # Avoid unnecessary recursion
        if min_dist_to_node > max_distance and len(best) == k:
            kth_distance = -best[0][0] if best else float('inf')
            if min_dist_to_node >= kth_distance:
                return best  
        
        # Recursively search in child containing the query drone first
        child_id = self.get_child_index(drone)
        if self.children[child_id]:
            best = self.children[child_id].find_k_nearest(drone, k, max_distance, best)
        
        # Create priority queue for other children ordered by minimum distance
        child_nodes = []
        for i in range(8):
            if i != child_id and self.children[i]:
                child_min_dist = self.children[i].min_distance_to_bounds(drone)
                if child_min_dist <= max_distance:
                    heapq.heappush(child_nodes, (child_min_dist, i))
        
        # Process other children in order of increasing minimum distance
        while child_nodes:
            min_dist, i = heapq.heappop(child_nodes)
            
            # Further pruning for each child node
            if len(best) == k:
                kth_distance = -best[0][0] if best else float('inf')
                if min_dist >= kth_distance:
                    continue 
            
            # Recursive conquer step
            best = self.children[i].find_k_nearest(drone, k, max_distance, best)
        
        # Merge
        return best
    
    def min_distance_to_bounds(self, drone: Drone) -> float:
        if drone.x < self.bounds['min_x']:
            dx = self.bounds['min_x'] - drone.x
        elif drone.x > self.bounds['max_x']:
            dx = drone.x - self.bounds['max_x']
        else:
            dx = 0  # Inside bounds in x
        
        if drone.y < self.bounds['min_y']:
            dy = self.bounds['min_y'] - drone.y
        elif drone.y > self.bounds['max_y']:
            dy = drone.y - self.bounds['max_y']
        else:
            dy = 0
        
        if drone.z < self.bounds['min_z']:
            dz = self.bounds['min_z'] - drone.z
        elif drone.z > self.bounds['max_z']:
            dz = drone.z - self.bounds['max_z']
        else:
            dz = 0
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def find_collision_pairs(self, safety_distance: float) -> List[Tuple[Drone, Drone, float]]:
        """
        Find all drone pairs closer than safety_distance using divide-and-conquer.
        
        Divide: Space partitioned into octants.
        Conquer: Check collisions within each octant recursively.
        Merge: Check collisions between drones in different nodes when necessary.
        """
        collision_pairs = []
        
        # Base case: Check collisions within leaf node
        if self.leaf:
            for i in range(len(self.drones)):
                for j in range(i + 1, len(self.drones)):
                    distance = self.drones[i].distance_to(self.drones[j])
                    if distance < safety_distance:
                        collision_pairs.append((self.drones[i], self.drones[j], distance))
            return collision_pairs
        
        # Conquer: Recursively find collisions in children
        for child in self.children:
            if child:
                # Recursive call to find intra-child collisions
                collision_pairs.extend(child.find_collision_pairs(safety_distance))
        
        # Merge: Check collisions between drones in different children
        for i in range(8):
            for j in range(i + 1, 8):
                if self.children[i] and self.children[j]:
                    # Calculate minimum distance between node bounding boxes
                    min_dist_between_nodes = self.min_distance_between_nodes(
                        self.children[i], self.children[j]
                    )
                    
                    # Only check inter-node collisions if nodes are close enough
                    if min_dist_between_nodes < safety_distance:
                        drones_i = self.children[i].collect_all_drones()
                        drones_j = self.children[j].collect_all_drones()
                        
                        # Check all pairs between the two sets
                        for drone_i in drones_i:
                            for drone_j in drones_j:
                                distance = drone_i.distance_to(drone_j)
                                if distance < safety_distance:
                                    collision_pairs.append((drone_i, drone_j, distance))
        
        return collision_pairs
    
    def min_distance_between_nodes(self, node1: 'TreeNode', node2: 'TreeNode') -> float:
        if node1.bounds['max_x'] < node2.bounds['min_x']:
            dx = node2.bounds['min_x'] - node1.bounds['max_x']
        elif node2.bounds['max_x'] < node1.bounds['min_x']:
            dx = node1.bounds['min_x'] - node2.bounds['max_x']
        else:
            dx = 0  
        
        if node1.bounds['max_y'] < node2.bounds['min_y']:
            dy = node2.bounds['min_y'] - node1.bounds['max_y']
        elif node2.bounds['max_y'] < node1.bounds['min_y']:
            dy = node1.bounds['min_y'] - node2.bounds['max_y']
        else:
            dy = 0
        
        if node1.bounds['max_z'] < node2.bounds['min_z']:
            dz = node2.bounds['min_z'] - node1.bounds['max_z']
        elif node2.bounds['max_z'] < node1.bounds['min_z']:
            dz = node1.bounds['min_z'] - node2.bounds['max_z']
        else:
            dz = 0
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)