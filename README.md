# Semantic SLAM

A Semantic SLAM system built on **ROS2 Humble** that fuses real-time object detection with LiDAR depth estimation to construct a semantically annotated map of the environment. The robot can answer spatial queries like *"Where is the chair?"* by maintaining a persistent memory of detected objects anchored to their map-frame coordinates.

---

## Demo

> Robot explores a Gazebo environment, detects objects via YOLOv8, fuses detections with LiDAR, and annotates the SLAM map with semantic markers in RViz2.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                        Gazebo Sim                        │
│   /camera/image_raw   /scan   /tf   /odom               │
└────────────┬──────────────┬────────────────────────────┘
             │              │
             ▼              ▼
    ┌─────────────────────────────┐
    │       perception_node        │
    │  YOLOv8n  +  LiDAR Fusion   │
    │  Camera-LiDAR angle mapping  │
    │  TF: base_link → map         │
    └──────────────┬──────────────┘
                   │  /semantic/observations
                   ▼
    ┌─────────────────────────────┐
    │      semantic_map_node       │
    │  Birikimli nesne hafızası    │
    │  Distance-threshold dedup    │
    │  /semantic/query  service    │
    └──────────┬──────────────────┘
               │  /semantic/map_data
               ▼
    ┌─────────────────────────────┐
    │      semantic_viz_node       │
    │  MarkerArray → RViz2         │
    │  Sphere + label overlay      │
    └─────────────────────────────┘
```

---

## Packages

| Package | Type | Description |
|---|---|---|
| `sim_bringup` | ament_cmake | Gazebo world, TB3 Waffle launch, SLAM Toolbox, teleop |
| `semantic_interfaces` | ament_cmake | Custom `SemanticObservation.msg`, `SemanticMap.msg`, `SemanticQuery.srv` |
| `perception` | ament_python | YOLOv8n detection + LiDAR fusion + TF transform |
| `semantic_map` | ament_python | Semantic map state, deduplication, query service |
| `semantic_viz` | ament_python | RViz2 MarkerArray visualization |

---

## Custom Interfaces

**`SemanticObservation.msg`**
```
string label
float32 confidence
geometry_msgs/PoseStamped pose
```

**`SemanticMap.msg`**
```
semantic_interfaces/SemanticObservation[] observations
```

**`SemanticQuery.srv`**
```
string object
---
semantic_interfaces/SemanticObservation[] observations
```

---

## Pipeline

### 1. Object Detection
YOLOv8n (COCO weights) processes `/camera/image_raw` frames. Each detection produces a bounding box center pixel `(u, v)` and a confidence score.

### 2. Camera → LiDAR Angle Mapping
The pixel center is converted to a bearing angle in the camera frame using the intrinsic matrix:
```
angle_camera = arctan2(u - cx, fx)
```
The camera-to-LiDAR frame rotation (from TF) is applied (`-arctan2` sign flip due to axis convention), then the corresponding LiDAR range is sampled from the angular window of the bounding box.

### 3. Polar → Map Frame
```
x = distance * cos(angle)
y = distance * sin(angle)
```
The resulting `PoseStamped` in `base_link` is transformed to `map` frame via `tf2_geometry_msgs`.

### 4. Semantic Map Deduplication
A new observation is stored only if it is farther than `distance_threshold` (default: `0.5m`) from all existing instances of the same label. This uses a `for/else` loop over each label's pose list.

### 5. Visualization
Each stored observation is rendered as a blue `SPHERE` marker with a `TEXT_VIEW_FACING` label offset `+0.3m` in Z, published as a `MarkerArray` to RViz2.

---

## Prerequisites

```bash
# ROS2 Humble
sudo apt install ros-humble-slam-toolbox \
                 ros-humble-turtlebot3-gazebo \
                 ros-humble-teleop-twist-keyboard \
                 ros-humble-tf2-geometry-msgs

# Python deps
pip install ultralytics --break-system-packages

# TurtleBot3 model
export TURTLEBOT3_MODEL=waffle
```

---

## Build & Run

```bash
cd ~/semantic_slam_ws
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
```

**Launch simulation + SLAM:**
```bash
ros2 launch sim_bringup slam_sim.launch.py
```

**Start perception:**
```bash
ros2 run perception perception_node --ros-args -p conf_threshold:=0.35
```

**Start semantic map:**
```bash
ros2 run semantic_map semantic_map_node
```

**Start visualization:**
```bash
ros2 run semantic_viz semantic_viz_node
```

---

## Querying the Semantic Map

```bash
# Find all chairs
ros2 service call /semantic/query semantic_interfaces/srv/SemanticQuery "{object: 'chair'}"

# Find all people
ros2 service call /semantic/query semantic_interfaces/srv/SemanticQuery "{object: 'person'}"
```

---

## Configuration

| Parameter | Node | Default | Description |
|---|---|---|---|
| `yolo_model_path` | perception | `yolov8n.pt` | Path to YOLO weights |
| `image_size` | perception | `640` | YOLO inference resolution |
| `conf_threshold` | perception | `0.35` | Detection confidence threshold |
| `distance_threshold` | semantic_map | `0.5` | Min distance (m) between distinct instances |

---

## Known Limitations

- **Sim-to-real domain gap**: YOLOv8 COCO weights struggle with Gazebo's non-photorealistic rendering. Lower `conf_threshold` (e.g. `0.05`) for simulation.
- **No depth camera**: Depth is estimated from 2D LiDAR by projecting the bbox center angle onto the scan. This can mis-estimate depth for thin or partially occluded objects.
- **Stateless persistence**: The semantic map resets when the node is restarted. JSON serialization can be added for persistence.

---

## Stack

- ROS2 Humble
- Python 3.10
- YOLOv8n (Ultralytics)
- SLAM Toolbox
- TurtleBot3 Waffle (Gazebo Classic)
- tf2 / tf2_geometry_msgs
- OpenCV + cv_bridge