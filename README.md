# Semantic SLAM + LLM-Guided Navigation

A full-stack autonomous navigation system built on **ROS2 Humble** that combines real-time semantic mapping with on-device natural language understanding. The robot builds a semantically annotated map as it explores, then accepts commands like *"Go to the refrigerator"* — parsing them with a local LLM, resolving them against the live semantic map, and executing navigation via Nav2.

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
    ┌─────────────────────────────┐        ┌──────────────────────────┐
    │      semantic_viz_node       │        │     llm_navigator_node    │
    │  MarkerArray → RViz2         │        │  Qwen2.5-0.5B (on-device) │
    │  Sphere + label overlay      │        │  NL command → label       │
    └─────────────────────────────┘        │  SemanticQuery client     │
                                           │  Nav2 NavigateToPose      │
                                           └──────────────────────────┘
                                                      ▲
                                           /llm_navigator/command
                                           (std_msgs/String)
```

---

## Packages

| Package | Type | Description |
|---|---|---|
| `sim_bringup` | ament_cmake | Gazebo world, TB3 Waffle launch, Nav2, teleop |
| `semantic_interfaces` | ament_cmake | Custom msg/srv definitions |
| `perception` | ament_python | YOLOv8n detection + LiDAR fusion + TF transform |
| `semantic_map` | ament_python | Semantic map state, deduplication, query service |
| `semantic_viz` | ament_python | RViz2 MarkerArray visualization |
| `llm_navigator` | ament_python | On-device LLM command parsing + Nav2 goal dispatch |

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
A new observation is stored only if it is farther than `distance_threshold` (default: `0.5m`) from all existing instances of the same label. Uses a `for/else` loop over each label's pose list.

### 5. Natural Language Navigation
User sends a command to `/llm_navigator/command`. Qwen2.5-0.5B-Instruct (running locally on GPU) extracts the target label via few-shot prompted text generation. The label is queried against the semantic map, and the nearest matching pose is sent as a `NavigateToPose` goal to Nav2.

### 6. Visualization
Each stored observation is rendered as a blue `SPHERE` marker with a `TEXT_VIEW_FACING` label offset `+0.3m` in Z, published as a `MarkerArray` to RViz2.

---

## Prerequisites

```bash
# ROS2 Humble
sudo apt install ros-humble-nav2-bringup \
                 ros-humble-turtlebot3-navigation2 \
                 ros-humble-turtlebot3-gazebo \
                 ros-humble-teleop-twist-keyboard \
                 ros-humble-tf2-geometry-msgs

# Python deps
pip install ultralytics transformers accelerate --user

export TURTLEBOT3_MODEL=waffle
```

---

## Build

```bash
cd ~/semantic_slam_ws
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
```

---

## Run

**Terminal 1 — Simulation:**
```bash
ros2 launch sim_bringup slam_sim.launch.py
```

**Terminal 2 — Navigation stack:**
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=~/semantic_slam_ws/maps/semantic_slam_map.yaml
```
> Set initial pose in RViz2 using **"2D Pose Estimate"** before proceeding.

**Terminal 3 — Semantic nodes:**
```bash
ros2 launch sim_bringup semantic_nodes.launch.py
```

**Terminal 4 — LLM Navigator:**
```bash
ros2 run llm_navigator llm_navigator_node
```

**Terminal 5 — Send a command:**
```bash
ros2 topic pub /llm_navigator/command std_msgs/String \
  "data: 'Go to the refrigerator near the wall'" --once
```

---

## Querying the Semantic Map Directly

```bash
ros2 service call /semantic/query semantic_interfaces/srv/SemanticQuery "{object: 'refrigerator'}"
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
- **No depth camera**: Depth is estimated from 2D LiDAR by projecting the bbox center angle onto the scan. Can mis-estimate depth for thin or partially occluded objects.
- **Stateless persistence**: Semantic map resets on node restart. JSON serialization can be added for persistence across sessions.
- **Single-instance navigation**: LLM navigator targets the first observed instance of a label. Multi-instance selection (e.g. "the chair on the left") is not yet implemented.

---

## Stack

- ROS2 Humble
- Python 3.10
- YOLOv8n (Ultralytics)
- Qwen2.5-0.5B-Instruct (HuggingFace Transformers)
- Nav2 + AMCL
- SLAM Toolbox
- TurtleBot3 Waffle (Gazebo Classic)
- tf2 / tf2_geometry_msgs
- OpenCV + cv_bridge