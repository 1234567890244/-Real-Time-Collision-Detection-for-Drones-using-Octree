# Real-Time Collision Detection for Drones using Octree

This project implements a divide-and-conquer algorithm based on an **octree** to efficiently detect collisions among numerous drones flying in a bounded urban airspace. It reduces the time complexity from \(O(n^2)\) to \(O(n \log n)\), enabling real‑time monitoring even for large drone swarms.

## Problem Description

- Each drone is represented by its 3D coordinates \((x_i, y_i, z_i)\).
- The system continuously monitors all drones and issues an immediate warning when any pair is closer than the **safety threshold** (20 meters).

## Motivation

With the rise of drone light shows, agricultural monitoring, and military operations, ensuring safe separation between many drones is critical. A naive pairwise check is too slow for large fleets. This project employs an **octree** – recursively subdividing the 3D space into eight equal subspaces – to organise drones hierarchically and accelerate collision detection.

## Algorithm Overview

### Core Idea

- **Divide**: Split the space along the midpoints of the X, Y, Z axes.
- **Conquer**: Recursively build subtrees in each subspace until leaf nodes contain at most \(C = 8\) drones.
- **Merge**: Combine collision results from subspaces, skipping cross‑node checks when bounding boxes are safely apart.

### Pseudocode

```text
BUILD_TREE(drones, bounds, depth):
    if len(drones) ≤ CAPACITY or depth ≥ MAX_DEPTH:
        self.drones = drones; return
    Compute midpoints for X, Y, Z axes
    Partition drones into 8 child nodes
    Recursively build tree for each non‑empty child

FIND_NEAREST(query_drone, k, max_dist):
    if leaf: brute‑force nearest in self.drones
    child_id = get_child_index(query_drone)
    nearest = children[child_id].find_nearest(...)
    Check adjacent child nodes if they may contain closer neighbors
    return nearest

FIND_COLLISION_PAIRS(safety_distance):
    if leaf: brute‑force pairwise comparison within leaf
    else:
        collisions = []
        for child in children:
            collisions.extend(child.find_collision_pairs(...))
        Merge: check drone pairs across child nodes if their bounding boxes are close
        return collisions
