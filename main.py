#!/usr/bin/env python3

import sys
import argparse
from typing import List, Tuple
from manager import DroneManager
from models import Drone, DroneStatus


def terminal_mode():
    print("=" * 60)
    print("Drone Collision Warning System")
    print("=" * 60)
    
    manager = DroneManager()
    
    print("\nStep 1: Input drone point set")
    print("Format: id, x_position (0-500), y_position (0-500), z_position (55-550)")
    print("Example: 001, 100.5, 200.3, 50.2")

    drones = []
    line_number = 1
    while True:
        user_input = input().strip()

        if user_input == '':
            if len(drones) < 2:
                print("At least 2 drones are required")
                continue
            break

        parts = user_input.split(',')
        if len(parts) != 4:
            print("Error: Input format should be: ID,x,y,z")
            continue
        
        drone_id = parts[0].strip()
        try:
            x = float(parts[1].strip())
            y = float(parts[2].strip())
            z = float(parts[3].strip())
        except ValueError:
            print("Error: Coordinates must be numbers")
            continue
        
        if any(d.id == drone_id for d in drones):
            print(f"Error: Drone ID '{drone_id}' already exists, please use a different ID")
            continue
        
        drone = Drone(
            id=drone_id,
            x=x, y=y, z=z,
            speed_x=0.0, speed_y=0.0, speed_z=0.0,
            status=DroneStatus.ACTIVE,
            is_manual=False
        )
        drones.append(drone)
        line_number += 1
    
    for drone in drones:
        manager.add_drone(drone)
    
    print("\nStep 2: Select the drone to query")
    print("Available drone IDs:")
    for i, drone in enumerate(drones, 1):
        print(f"  {i}. {drone.id} - position: ({drone.x}, {drone.y}, {drone.z})")
    
    while True:
        query_id = input("\nEnter the drone ID to query: ").strip()
        
        query_drone = manager.get_drone(query_id)
        if query_drone:
            break
        else:
            print(f"Error: Drone with ID '{query_id}' not found")
            print("Please select a valid drone ID from the list above")
    
    print(f"\nChecking warnings for drone '{query_id}'...")
    neighbors = manager.get_nearest_neighbors(query_id, k=1)
    
    if neighbors:
        distance, nearest_drone = neighbors[0]
        
        print("\n" + "=" * 60)
        print("Query Result:")
        print("=" * 60)
        print(f"Query drone: {query_id}")
        print(f"  Position: ({query_drone.x:.2f}, {query_drone.y:.2f}, {query_drone.z:.2f})")
        print(f"Nearest neighbor drone: {nearest_drone.id}")
        print(f"  Position: ({nearest_drone.x:.2f}, {nearest_drone.y:.2f}, {nearest_drone.z:.2f})")
        print(f"Distance: {abs(distance):.2f} meters")
        print("=" * 60)

    else:
        print("No warnings")
    
    print("\n" + "-" * 60)
    while True:
        choice = input("Continue querying other drones? (y/n): ").strip().lower()
        if choice == 'y':
            terminal_mode()
            return
        elif choice == 'n':
            return
        else:
            print("Please enter 'y' or 'n'")

if __name__ == "__main__":
    terminal_mode()