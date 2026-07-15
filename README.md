# Hybrid A* Global Planner — ROS 2 Nav2 Plugin with Reeds-Shepp Kinematics

**A ROS 2 nav2 global planner plugin implementing Hybrid A* path planning with Reeds-Shepp motion primitives, shot-to-goal analytic expansion, and path smoothing — developed and tested on an Autonomous Surface Vehicle (ASV) at the University of Michigan.**

![Demo](hybrid_astar_demo.gif)
<img width="640" height="360" alt="hybrid_astar_demo" src="https://github.com/user-attachments/assets/871da38d-74c6-4155-ad61-0b464238d1af" />

---

## Overview

Standard grid-based planners (A*, Dijkstra) produce paths that ignore vehicle heading and kinematic constraints, making them unsuitable for non-holonomic robots such as cars, boats, and ASVs. Hybrid A* solves this by searching in a continuous (x, y, θ) state space while still using a discrete grid for collision checking, guaranteeing that the resulting path is kinematically feasible from the start.

This implementation wraps Hybrid A* as a drop-in ROS 2 Nav2 global planner plugin. It uses **Reeds-Shepp curves** (via OMPL) as the motion model, enabling the planner to reason about both forward and reverse motion with smooth curvature. A **shot-to-goal analytic expansion** attempts a direct Reeds-Shepp connection to the goal at every node, dramatically reducing search time near the goal. A post-processing **path smoother** removes discretization artifacts before the path is sent to the local planner.

This project was developed as an independent research study at the University of Michigan, applied to an ASV (Autonomous Surface Vehicle) platform.

---

## Key Features

- **Nav2 plugin interface** — drop-in replacement for NavFn or Theta*; no changes to the rest of the navigation stack required
- **Reeds-Shepp motion model** — kinematically feasible primitives via OMPL; supports both forward and reverse motion
- **Shot-to-goal analytic expansion** — at each expanded node, attempts a direct Reeds-Shepp connection to the goal; exits search early when feasible
- **Path smoother** — post-processing step to remove grid-discretization jitter before commanding the robot
- **72-bin angle discretization** — 5° resolution heading bins balancing planning resolution and memory footprint
- **11 steering directions** — delta angles from -5 to +5 bins per expansion step for smooth curve generation
- **Occupancy grid collision checking** — configurable obstacle threshold and unknown-cell handling
- **ASV-validated** — designed and tested on a surface vehicle platform with large turning radii and open-water environments

---

## System Architecture

```
ROS 2 Nav2 Stack
       |
  GlobalPlanner (plugin)
       |
  HybridAStar::makePlan()
       |
  +----+---------------------------+
  |                                |
  Priority Queue                Shot-to-Goal
  (f = g + h)                   (Reeds-Shepp
       |                         direct connection)
  Reeds-Shepp Move                |
  (OMPL motion primitive)     If feasible -> return path
       |
  Collision Check
  (OccupancyGrid)
       |
  Node expansion (11 steering angles)
       |
  Goal reached? -> reconstruct path
       |
  Path Smoother
       |
  nav_msgs::msg::Path -> LocalPlanner
```

**Algorithm flow:**
1. Convert start/goal world poses to grid indices and angle bins
2. Initialize priority queue with the start node (f = Euclidean distance to goal)
3. At each iteration: pop lowest-f node, attempt shot-to-goal, expand 11 steering directions
4. Each expansion uses a Reeds-Shepp move primitive; discard if collision detected
5. On goal reach or successful shot: reconstruct path by walking parent pointers
6. Apply path smoother and publish `nav_msgs::msg::Path`

---

## Requirements

| Dependency | Version |
|---|---|
| ROS 2 | Humble |
| Nav2 | matching ROS 2 version |
| OMPL | 1.5+ (for ReedsSheppStateSpace) |
| C++ | 17 |
| CMake | 3.16+ |

**Ubuntu 22.04 / 24.04** recommended.

---

## Installation

```bash
# 1. Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/yslin0524/Hybrid_A-.git

# 2. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build --symlink-install --packages-select hybrid_a_star_planner
source install/setup.bash
```

---

## Usage

### Loading as a Nav2 plugin

In your Nav2 params YAML, set the global planner plugin:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "hybrid_a_star_planner::HybridAStarPlanner"
      allow_unknown: true
      obstacle_threshold: 65
```

Then launch your Nav2 stack as usual:

```bash
ros2 launch nav2_bringup navigation_launch.py params_file:=your_params.yaml
```

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `allow_unknown` | `true` | Whether to treat unknown cells as free |
| `obstacle_threshold` | `65` | OccupancyGrid cost value above which a cell is treated as an obstacle (0-100) |
| `num_angle_bins` | `72` | Number of heading discretization bins (72 = 5° resolution) |
| `angle_bin_size` | `2π/72` | Angular resolution per bin (auto-computed) |

---

## Application

This planner was developed and validated during a one-semester independent research study at the **University of Michigan**, applied to an **Autonomous Surface Vehicle (ASV)**. The ASV setting motivated several design choices:

- **Large turning radii** — Reeds-Shepp curves naturally encode minimum-turn-radius constraints, critical for surface vessels
- **Open-water environments** — sparse obstacle maps with large free-space regions make shot-to-goal particularly effective, as direct connections succeed frequently
- **No LiDAR** — the planner operates on any occupancy grid source, including camera-derived or map-based grids

---

## References

```bibtex
@inproceedings{dolgov2008practical,
  title     = {Practical Search Techniques in Path Planning for Autonomous Driving},
  author    = {Dolgov, Dmitri and Thrun, Sebastian and Montemerlo, Michael and Diebel, James},
  booktitle = {AAAI Workshop on Artificial Intelligence and Robotics},
  year      = {2008}
}

@inproceedings{reeds1990optimal,
  title   = {Optimal paths for a car that goes both forwards and backwards},
  author  = {Reeds, James and Shepp, Lawrence},
  journal = {Pacific Journal of Mathematics},
  volume  = {145},
  number  = {2},
  pages   = {367--393},
  year    = {1990}
}
```

---

## License

MIT
