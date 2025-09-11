# Pontus ROS2 Stack

Pontus is a collection of ROS 2 packages that power our autonomous underwater vehicle (AUV) for the RoboSub competition. The stack is organised as a set of modular packages that handle perception, localization, mapping, control and high-level autonomy. This document gives new developers a high-level overview of each package, the interfaces they expose and how the pieces fit together to achieve autonomous behaviour.

---

## `pontus_bringup`

### Overview
`pontus_bringup` is a collection of launch files that orchestrate the startup of the Pontus software stack. These launch files decide which subsystems are included (simulation vs hardware, controller type, localization, sensors) and pass in appropriate configuration arguments. They do not directly publish or subscribe to topics --- instead, they combine nodes from other packages.

### Launch Files

#### `auv.launch.py`
- **Purpose:** Main hardware bringup for the AUV.
- **Includes:**
  - `pontus_description/spawn.launch.py` --- loads the URDF/Xacro and publishes TF tree.
  - `pontus_controller/los_control.launch.py` --- line-of-sight controller for waypoint navigation.
  - `pontus_localization/localization.launch.py` --- EKF-based localization.
- **Arguments:**
  - `auv` (default: `auv`) --- selects whether the vehicle is real hardware (auv) or simulation (sim)
  - `gazebo` (default: `False`) --- whether to spawn into Gazebo.

---

#### `direct_auv.launch.py`
- **Purpose:** Bringup with **direct controller** (raw thruster commands).
- **Includes:**
  - `pontus_description/spawn.launch.py`
  - `pontus_controller/direct_control.launch.py`
- **Nodes:**
  - `pontus_sensors/depth_republish.py` --- publishes depth topic.
- **Arguments:**
  - `auv` (default: `auv`)
  - `gazebo` (default: `False`)

---

#### `hybrid_auv.launch.py`
- **Purpose:** Bringup with **hybrid controller** (direct control + automatic depth hold).
- **Includes:**
  - `pontus_description/spawn.launch.py`
  - `pontus_controller/hybrid_control.launch.py`
  - `pontus_localization/localization.launch.py`
- **Nodes:**
  - `pontus_sensors/depth_republish.py`
- **Arguments:**
  - `auv` (default: `auv`)
  - `gazebo` (default: `False`)

---

#### `simulation.launch.py`
- **Purpose:** Standard Gazebo simulation environment with odometry + controller.
- **Includes:**
  - `pontus_sim/sim.launch.py` --- Gazebo world (default `underwater.world`).
  - `pontus_description/spawn.launch.py` --- spawns robot model in sim.
  - `pontus_localization/localization.launch.py` --- simulated localization.
  - `pontus_controller/vel_control.launch.py` --- velocity controller.
- **Arguments:**
  - `auv` --- whether real hardware (auv) or simulation (sim).
  - `world` (default: `underwater.world`) --- Gazebo world file.
  - `static` (default: `false`) --- static spawn toggle.

---

#### `odom_simulation.launch.py`
- **Purpose:** Gazebo sim with **odometry bridging** and LOS controller.
- **Includes:**
  - `pontus_sim/sim.launch.py` --- Gazebo world (default `prequal.world`).
  - `pontus_description/spawn.launch.py`
  - `pontus_sim/odom_bridge.launch.py` --- bridges Gazebo odometry into ROS topics.
  - `pontus_controller/los_control.launch.py` (with `sim: true`)
  - `pontus_localization/localization.launch.py` (`auv: sim`)
- **Nodes:**
  - `pontus_sensors/dvl_republish_sim.py` --- converts sim odometry into DVL-like messages.
- **Arguments:**
  - `world` (default: `prequal.world`)
  - `static` (default: `false`)

---

#### `odom_simulation_slalom.launch.py`
- **Purpose:** Gazebo sim for **slalom task** with odometry bridging and position control.
    - NOTE: this is effectivley the same as running `odom_simulation.launch.py` with the Gazebo world set to slalom.world
- **Includes:**
  - `pontus_sim/sim.launch.py` --- Gazebo world (default `slalom.world`).
  - `pontus_description/spawn.launch.py`
  - `pontus_sim/odom_bridge.launch.py`
  - `pontus_controller/pos_control.launch.py` (with `sim: true`)
  - `pontus_localization/localization.launch.py` (`auv: sim`)
- **Nodes:**
  - `pontus_sensors/dvl_republish_sim.py`
- **Arguments:**
  - `world` (default: `slalom.world`)
  - `static` (default: `false`)

---

### Scripts

- **`record_all_topics.sh`** — helper script to record all ROS 2 topics into a bag file for debugging and data analysis.

---

### Interfaces
The `pontus_bringup` package itself does **not** expose topics or services. Instead, it composes:
- `pontus_description` (robot model + TF)
- `pontus_controller` (various controllers)
- `pontus_localization` (EKF odometry)
- `pontus_sensors` (depth/DVL republishers)
- `pontus_sim` (Gazebo + odom bridge in simulation)

---

## `pontus_sensors`

### Overview
The `pontus_sensors` package contains sensor interface nodes. Each node reads raw data from hardware (or simulation), repackages it into standard ROS messages, and republishes it on namespaced topics (`/pontus/...`). This ensures consistent message types and frame conventions across the stack.

---

### Depth Sensor

Barometric pressure sensor that provides depth relative to the surface. Data is converted from pressure into meters and published as an `Odometry` message with depth encoded in the Z-axis.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/depth_sensor` | Topic | `std_msgs/Float32` | Raw barometric pressure from depth sensor |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/depth_0` | Topic | `nav_msgs/Odometry` | Depth estimate in meters (Z position) |

---

### Doppler Velocity Log (DVL)

Provides vehicle velocity and dead-reckoned odometry. Two republishers exist: one for real hardware (`dvl_republish.py`) that transforms frames, and one for simulation (`dvl_republish_sim.py`) that adapts Gazebo’s odometry.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/dvl/odometry` | Topic | `nav_msgs/Odometry` | Raw odometry output from the DVL or Gazebo sim |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/dvl` | Topic | `nav_msgs/Odometry` | Corrected odometry in the `odom` -> `dvl_a50_link` frame |

---

### Inertial Measurement Unit (IMU)

Provides accelerometer and gyroscope data for orientation and angular velocity. Republished with corrected frame.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/imu/data` | Topic | `sensor_msgs/Imu` | Raw IMU data from sensor |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/imu_0` | Topic | `sensor_msgs/Imu` | Republished IMU data in `imu_0` frame |

---

### Cameras

Stereo/monocular cameras are republished into standardized topics for downstream vision modules. Generic republish node takes an `input` topic and republishes to `output`. These are always renamed to something more useful (check `pontus_perception` for more).

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `input` | Topic | `sensor_msgs/Image` | Raw camera image (node parameterized per camera) |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `output` | Topic | `sensor_msgs/Image` | Republished camera image for perception stack |

---

### Sonoptix Imaging Sonar

Forward-looking sonar driver that connects via RTSP and API. Captures grayscale sonar returns, preprocesses them, and republishes as ROS `Image` messages. We then have a node in `pontus_perception` which will convert this into a useable point cloud.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| N/A | --- | --- | Reads directly from RTSP stream |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/sonar_0/image_debug` | Topic | `sensor_msgs/Image` | Preprocessed sonar image (grayscale) |


---


---

## `pontus_perception`

### Overview
`pontus_perception` ingests camera and sonar streams, runs detection/geometry pipelines, and republishes standardized outputs (detections, images, point clouds). Most topic names and camera IDs are configured via:
- `config/auv/*.yaml` when running on hardware
- `config/sim/*.yaml` when running in simulation  
Primary launch files:
- `launch/perception.launch.py` — boots the full perception stack (uses AUV vs SIM configs)
- `launch/yolo.launch.py` — boots only the YOLO pipeline

> **Naming note:** In the tables below, `CAMERA_<id>` means a specific camera name from your YAML (e.g., `camera_left`, `camera_right`, `camera_0`). Replace with your actual IDs.

---

### Vision: YOLO Object Detection

Runs a YOLO model on a specified camera stream and publishes detection messages for downstream autonomy/mapping. Model paths are in `yolo/auv/model.pt` and `yolo/sim/model.pt`; parameters are in `config/*/yolo.yaml`.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/CAMERA_<id>/image_raw` **or** `/pontus/CAMERA_<id>/image_raw/compressed` | Topic | `sensor_msgs/Image` **or** `sensor_msgs/CompressedImage` | Input image stream for YOLO inference |
| `/pontus/CAMERA_<id>/camera_info` | Topic | `sensor_msgs/CameraInfo` | Intrinsics (used by pose heads / geometry) |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/CAMERA_<id>/yolo_results` | Topic | `pontus_msgs/YOLOResultArray` | 2D detection boxes/classes/scores |
| `/pontus/CAMERA_<id>/yolo_overlay` *(optional)* | Topic | `sensor_msgs/Image` | Debug image with drawn boxes (if enabled in YAML) |

---

### Vision: YOLO Pose / 3D Geometry

If enabled, converts YOLO boxes + camera intrinsics into approximate 3D rays/poses for targets (e.g., gate poles, buoys). Parameters in `config/*/yolo.yaml`.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/CAMERA_<id>/yolo_results` | Topic | `pontus_msgs/YOLOResultArray` | 2D detections to convert |
| `/pontus/CAMERA_<id>/camera_info` | Topic | `sensor_msgs/CameraInfo` | Intrinsics for projection |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/CAMERA_<id>/yolo_target_pose` | Topic | `geometry_msgs/PoseArray` or `geometry_msgs/PoseStamped` | Approx. target poses/rays for downstream use |
| `/pontus/yolo_target_markers` *(optional)* | Topic | `visualization_msgs/MarkerArray` | RViz markers for debugging |

---

### Vision: Color Threshold Detection (Slalom / Color Cues)

Fast classical vision pass to find color-coded structures (e.g., slalom poles, gate elements). Tunables in `config/*/gate_detection.yaml`.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/CAMERA_<id>/image_raw` | Topic | `sensor_msgs/Image` | Input RGB image |
| `/pontus/CAMERA_<id>/camera_info` | Topic | `sensor_msgs/CameraInfo` | Intrinsics (optional, for geometry helpers) |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/slalom_detections` | Topic | `pontus_msgs/SlalomDetectorResults` | Detected slalom poles / color cues |
| `/pontus/slalom_debug_image` *(optional)* | Topic | `sensor_msgs/Image` | Thresholded / annotated debug image |

---

### Vision: Stereo Geometry / Paired-Camera Helpers

Stereo utilities (rectification/geometry) to produce baselines for depth estimates or to improve pose from paired cameras. Defaults defined in `config/*/camera_config.yaml`.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/CAMERA_left/image_raw` | Topic | `sensor_msgs/Image` | Left camera image |
| `/pontus/CAMERA_right/image_raw` | Topic | `sensor_msgs/Image` | Right camera image |
| `/pontus/CAMERA_left/camera_info` | Topic | `sensor_msgs/CameraInfo` | Left intrinsics/extrinsics |
| `/pontus/CAMERA_right/camera_info` | Topic | `sensor_msgs/CameraInfo` | Right intrinsics/extrinsics |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/stereo/keypoints` *(optional)* | Topic | custom or `sensor_msgs/Image` | Debug matches/rectified views |
| `/pontus/stereo/targets` *(optional)* | Topic | `geometry_msgs/PoseArray` | Triangulated target estimates |

---

### Sonar: Polar -> Cartesian Rectification

Converts a forward-looking sonar’s polar scan to a Cartesian, image-like grid for downstream CV. Tunables (angle/range) typically live alongside sonar config; perception node publishes rectified frames.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/sonar_0/image_polar` *(name may vary)* | Topic | `sensor_msgs/Image` | Polar sonar frame |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/sonar_0/image_rect` | Topic | `sensor_msgs/Image` | Cartesian rectified sonar image |
| `/pontus/sonar_0/image_debug` *(optional)* | Topic | `sensor_msgs/Image` | Intermediate/annotated view |

---

### Point Cloud: Downsampling

Generic point cloud voxel grid/downsample to reduce bandwidth/CPU for mapping/perception fusers.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/points_in` | Topic | `sensor_msgs/PointCloud2` | Raw/high-density cloud |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/points_downsampled` | Topic | `sensor_msgs/PointCloud2` | Downsampled cloud for fusion/visualization |

---

### Shape Detection: Cylindrical Targets

Detect cylindrical structures (e.g., vertical markers / transponders) in camera or sonar frames depending on the configured source.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/CAMERA_<id>/image_raw` **or** `/pontus/sonar_0/image_rect` | Topic | `sensor_msgs/Image` | Source image for cylinder detection |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/cylinder_detections` | Topic | custom or `geometry_msgs/PoseArray` | Detected cylinder positions/poses |
| `/pontus/cylinder_debug_image` *(optional)* | Topic | `sensor_msgs/Image` | Annotated detection overlay |

---

### Camera Utilities

Small helpers to normalize camera I/O across the stack.

#### Compressed Image Republisher
- **Subscribed:** `/pontus/CAMERA_<id>/image_raw/compressed` (`sensor_msgs/CompressedImage`)
- **Published:** `/pontus/CAMERA_<id>/image_raw` (`sensor_msgs/Image`) — decoded stream for nodes that require raw images

#### Camera Info Publishers
- **Published:**  
  - `/pontus/camera_left/camera_info` (`sensor_msgs/CameraInfo`)  
  - `/pontus/camera_right/camera_info` (`sensor_msgs/CameraInfo`)  
- **Purpose:** Publish calibrated intrinsics/extrinsics from `config/*/camera_config.yaml` if the physical driver doesn’t supply them.

---

### Tools / Off-line: Bag to MP4

Utility node to dump an image stream from a ROS bag to an `.mp4` file for debugging, demos, and dataset curation.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/CAMERA_<id>/image_raw` **or** `/pontus/CAMERA_<id>/image_raw/compressed` | Topic | `sensor_msgs/Image` or `sensor_msgs/CompressedImage` | Input frames to record |

#### Published Topics / Services
| Name | Type | Message/Service | Purpose |
| --- | --- | --- | --- |
| --- | --- | --- | Writes video file to disk; no ROS outputs |

---

### Parameters & Config Files

#### `config/*/camera_config.yaml`
- **What:** Camera frames, names/IDs, calibration (intrinsics, distortion), stereo baseline/extrinsics.
- **Used by:** `camera_info_publisher_*`, stereo helpers, YOLO pose projection.

#### `config/*/yolo.yaml`
- **What:** Model path, confidence/NMS thresholds, class filters, image size, input topic selection (raw vs compressed).
- **Used by:** `yolo_node.py`, `yolo_pose_detection.py`.

#### `config/*/gate_detection.yaml`
- **What:** HSV/range thresholds, morphology options, min/max area/aspect filters, debug overlays.
- **Used by:** `color_threshold_detection.py`, `stereo_detection.py` (if configured).

---

### Launch Files

#### `launch/perception.launch.py`
Boots the **full perception** graph using either `config/auv/*` or `config/sim/*`. Typical inclusions:
- YOLO detector (+ optional pose node)
- Color threshold detector
- (Optional) stereo helpers
- Camera info publishers / compressed republisher
- Sonar polar→rect node (if sonar enabled)
- Point cloud downsampling (if cloud source configured)

Key launch arguments (typical):
- `use_sim_time` (bool)
- `platform` (`auv` or `sim`)
- `camera_ids` / topic remaps (strings)
- Paths to `camera_config.yaml`, `yolo.yaml`, `gate_detection.yaml`

#### `launch/yolo.launch.py`
Starts **only** the YOLO pipeline with its config bundle; useful for tuning and benchmarking. Typical args:
- `platform` (`auv` or `sim`)
- `camera_id`
- `yolo_config` (path to YAML)
- Optional image transport selection (raw vs compressed)

---

### Quick Topic Summary (replace IDs with your actual names)

| Sensor / Node | Subscribes | Publishes |
| --- | --- | --- |
| YOLO | `/pontus/CAMERA_<id>/image_raw(*/compressed)`, `/pontus/CAMERA_<id>/camera_info` | `/pontus/CAMERA_<id>/yolo_results`, `/pontus/CAMERA_<id>/yolo_overlay` (opt) |
| YOLO Pose | `/pontus/CAMERA_<id>/yolo_results`, `/pontus/CAMERA_<id>/camera_info` | `/pontus/CAMERA_<id>/yolo_target_pose`, `/pontus/yolo_target_markers` (opt) |
| Color Threshold | `/pontus/CAMERA_<id>/image_raw`, `/pontus/CAMERA_<id>/camera_info` | `/pontus/slalom_detections`, `/pontus/slalom_debug_image` (opt) |
| Stereo Helpers | Left/Right `image_raw` + `camera_info` | `/pontus/stereo/targets`, `/pontus/stereo/keypoints` (opt) |
| Sonar Rectifier | `/pontus/sonar_0/image_polar` | `/pontus/sonar_0/image_rect`, `/pontus/sonar_0/image_debug` (opt) |
| PC Downsample | `/pontus/points_in` | `/pontus/points_downsampled` |
| Compressed Republisher | `.../image_raw/compressed` | `.../image_raw` |
| CameraInfo Pubs | --- | `.../camera_info` |
| Bag to MP4 | `.../image_raw(*/compressed)` | (file only) |

---

## `pontus_localization`

### Overview
`pontus_localization` provides state estimation for the vehicle by fusing sensor inputs into a single odometry output using the [`robot_localization`](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html) package.  
It runs an EKF (`ekf_node`) configured separately for AUV hardware vs simulation via YAML files in `config/`.

---

### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/dvl` | Topic | `nav_msgs/Odometry` | Dead-reckoned odometry from DVL republisher |
| `/pontus/imu_0` | Topic | `sensor_msgs/Imu` | IMU orientation/angular velocity |
| `/pontus/depth_0` | Topic | `nav_msgs/Odometry` | Depth estimate (Z position) from pressure sensor |
| `/pontus/gps` *(optional, sim only)* | Topic | `sensor_msgs/NavSatFix` | GPS position (if enabled in sim config) |

*(Exact inputs depend on the `ekf.yaml` configuration in `config/auv` or `config/sim`.)*

---

### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/odometry` | Topic | `nav_msgs/Odometry` | Fused 6-DOF odometry used throughout the stack (controllers, mapping, autonomy) |
| `/tf` | Topic | `tf2_msgs/TFMessage` | Broadcasts `odom -> base_link` transform |

---

### Parameters & Config Files
- `config/auv/ekf.yaml`  
  EKF configuration for real hardware. Fuses DVL, IMU, and depth.
- `config/sim/ekf.yaml`  
  EKF configuration for simulation. May also fuse Gazebo-perfect odometry and optional GPS.

Key EKF parameters defined:
- `frequency` (Hz update rate)  
- `sensor_timeout`  
- `odom0`, `imu0`, `pose0` inputs  
- Which state variables (x, y, z, roll, pitch, yaw, velocities, accelerations) are estimated.

---

### Launch Files

#### `localization.launch.py`
- Starts an EKF filter node from `robot_localization`.
- Selects configuration (`config/auv/ekf.yaml` or `config/sim/ekf.yaml`) based on the `auv` argument.
- Remaps the default `/odometry/filtered` output to `/pontus/odometry`.

**Arguments:**
- `auv` --- selects AUV namespace/config (`auv` or `sim`).

---

---

## `pontus_mapping`

### Overview
`pontus_mapping` maintains higher-level spatial representations of the RoboSub competition environment. It has two main components:

1. **Semantic Map Manager** — stores and updates detected objects (gates, markers, octagon, etc.) with confidence scores. Provides services for autonomy tasks to query object positions.
2. **Exploration Map Manager** — maintains an occupancy grid of explored vs unexplored areas, enabling region-based search behaviors.

---

### Semantic Map Manager

Manages a database of detected semantic objects (gates, slalom poles, vertical marker, octagon). Fuses multiple detections, removes duplicates, and publishes RViz/Foxglove markers.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/odometry` | Topic | `nav_msgs/Odometry` | Robot pose used to filter detections (ignore while moving, above surface) |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/semantic_map_visual` | Topic | `visualization_msgs/MarkerArray` | Visual markers of detected objects (meshes for shark/fish, cylinders for poles, etc.) |

#### Services
| Name | Type | Service | Purpose |
| --- | --- | --- | --- |
| `/pontus/add_semantic_object` | Service | `pontus_msgs/AddSemanticObject` | Add YOLO or threshold-based detections into the map |
| `/pontus/gate_side_information` | Service | `pontus_msgs/GateSideInformation` | Record/query which side of the gate was entered, also determine fish/shark sign orientation |
| `/pontus/get_gate_detection` | Service | `pontus_msgs/GetGateLocation` | Query current estimated left/right gate pole locations |
| `/pontus/get_vertical_marker_detection` | Service | `pontus_msgs/GetVerticalMarkerLocation` | Query current vertical marker location |

---

### Exploration Map Manager

Maintains an `OccupancyGrid` representing explored vs unexplored areas. Integrates odometry and FOV helpers to gradually mark visited cells. Provides an action server to command region searches.

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/odometry` | Topic | `nav_msgs/Odometry` | Used to update which cells of the map have been visited |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/exploration_map` | Topic | `nav_msgs/OccupancyGrid` | Map of explored vs unexplored cells |

#### Actions
| Name | Type | Action | Purpose |
| --- | --- | --- | --- |
| `/pontus/search_region` | Action | `pontus_msgs/SearchRegion` | Command the AUV to search within a polygonal region |

---

### Helpers

- **`helpers.py`**  
  - `get_fov_polygon()` — builds polygon from odometry + FOV params.  
  - `polygon_contained()` — point-in-polygon check for filtering detections.
- **Visual meshes** (`visual_meshes/`) — OBJ/MTL models of gates, markers, and signs for richer visualization in RViz/Foxglove.

---

### Config Files

- **`config/visited_parameters.yaml`** — parameters controlling exploration FOV angle, range, grid resolution, and map size.

---

### Launch Files

#### `mapping.launch.py`
- Starts both:
  - `semantic_map_manager`  
  - `exploration_map_manager`  
- Declares parameters from `visited_parameters.yaml`.

---

### Interfaces Summary

| Interface | Type | Purpose |
| --- | --- | --- |
| `/pontus/odometry` | Topic (`nav_msgs/Odometry`) | Shared input for semantic + exploration maps |
| `/pontus/semantic_map_visual` | Topic (`visualization_msgs/MarkerArray`) | RViz visualization of semantic detections |
| `/pontus/exploration_map` | Topic (`nav_msgs/OccupancyGrid`) | Exploration grid visualization |
| `/pontus/add_semantic_object` | Service (`pontus_msgs/AddSemanticObject`) | Add detections to semantic map |
| `/pontus/gate_side_information` | Service (`pontus_msgs/GateSideInformation`) | Gate side info |
| `/pontus/get_gate_detection` | Service (`pontus_msgs/GetGateLocation`) | Query gate poles |
| `/pontus/get_vertical_marker_detection` | Service (`pontus_msgs/GetVerticalMarkerLocation`) | Query vertical marker |
| `/pontus/search_region` | Action (`pontus_msgs/SearchRegion`) | Exploration of polygonal regions |

---


## `pontus_mapping`

### Overview
Creates semantic map to record detected objects. Provides services and actions for autonomy to query and reason about the environment.

### Subscribed Topics / Services
| Name | Type | Message/Service | Purpose |
| --- | --- | --- | --- |
| `/pontus/odometry` | Topic | `nav_msgs/Odometry` | Postion awareness for mapping nodes |

### Published Topics / Services
| Name | Type | Message/Service | Purpose |
| --- | --- | --- | --- |
| `/pontus/semantic_map_visual` | Topic | `visualization_msgs/MarkerArray` | Visualisation of semantic objects |
| `/pontus/exploration_map` | Topic | `nav_msgs/OccupancyGrid` | Occupancy grid of explored areas (TODO: Check if this actually works) |
| `/pontus/add_semantic_object` | Service | `AddSemanticObject` | Insert detected object into semantic map |
| `/pontus/gate_side_information` | Service | `GateSideInformation` | Record/query which side of the gate was entered |
| `/pontus/get_gate_detection` | Service | `GetGateLocation` | Retrieve gate location estimates |
| `/pontus/get_vertical_marker_detection` | Service | `GetVerticalMarkerLocation` | Retrieve vertical marker location |
| `/pontus/search_region` | Action | `SearchRegion` | Command exploration of a polygonal region |

---

## `pontus_controller`

### Overview
Converts commanded body velocity into thruster forces, sends to the microcontroller. Supports manual joystick, estop, and depth feedback.

### Subscribed Topics / Services
| Name | Type | Message/Service | Purpose |
| --- | --- | --- | --- |
| `/cmd_vel` | Topic | `geometry_msgs/Twist` | Desired body velocity command |
| `/cmd_accel` | Topic | `geometry_msgs/Twist` | Body acceleration input |
| `/pontus/odometry` | Topic | `nav_msgs/Odometry` | Feedback odometry |
| `/autonomy_mode` | Topic | `std_msgs/Bool` | Autonomy switch state |
| `/pontus/depth_0` | Topic | `std_msgs/Float64` | Depth measurement |
| `/joy` | Topic | `sensor_msgs/Joy` | Joystick input |
| `/estop` | Topic | `std_msgs/Bool` | Emergency stop state |

### Published Topics / Services
| Name | Type | Message/Service | Purpose |
| --- | --- | --- | --- |
| `/pontus/thruster_<id>/cmd_thrust` | Topic | `std_msgs/Float64` | Individual thruster effort commands |
| `/pontus/go_to_pose` | Service | `pontus_msgs/GoToPose` | Send target pose for navigation |


### Launch Files
`pontus_controller` has several different launch files to be aware of as they launch different types of controllers for different purposes:

* `direct_controller.launch.py`: This is what you will use if you need to send thruster commands directly. Realistically should ontly be for esting and RC controls (such as when we were teesting if the robot could roll). You should not be sending direct thruster commands in autonomy unless there is a very good reason to do so.
* `hybrid_controller.launch.py`: Mostly the same as `direct_controller.launch.py` but it automatically maintains its depth.
* `los_controller.launch.py`: "line of sight" controller. When it is sent a waypoint to navigate to, this controller will first rotate the robot to be facing the point before driving in a straight line to it.

Other than the different types of controllers we also have a helper launch file to setup RC testing:
* `rc.launch.py`: Will launch the nodes to send RC commands to the rboot over the tether. NOTE: This should be run locally on the laptop used to monitor the robot, not on the robot itself.

---

## `pontus_autonomy`

### Overview
The `pontus_autonomy` package encodes the **high-level behaviors** that allow Pontus to complete RoboSub tasks (Gate, Slalom, Path Marker, etc.). Autonomy is structured as:

- **Tasks** (`pontus_autonomy/tasks/*`): encapsulated state machines for each competition task.
- **Runs** (`*_run.py`): sequences of tasks assembled into full mission scripts (prequalification, semi-final, slalom-only, etc.).
- **Helpers** (`pontus_autonomy/helpers/*`): abstractions for various tools to help in autoonmy tasks (GoToPose client, semantic map seeding, etc.)
- **Visualization** (`run_info_visualizer.launch.py`): publishes planned waypoints and semantic objects into RViz/Foxglove when creating a preplanned waypointed mission.

Autonomy tasks rely heavily on:
- **Perception outputs** (YOLO, slalom detector, etc.)
- **Mapping services** (semantic map, gate/marker info)
- **Controller actions** (`/pontus/go_to_pose`, `/pontus/search_region`)
- **Vehicle state** (`/pontus/odometry`, `/auto_enable`)

---

### Core Interfaces

#### Subscribed Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/pontus/odometry` | Topic | `nav_msgs/Odometry` | Vehicle pose feedback used in nearly all tasks |
| `/pontus/camera_<id>/yolo_results` | Topic | `pontus_msgs/YOLOResultArray` | Used for object detection |
| `/pontus/slalom_detector/results` | Topic | `pontus_msgs/SlalomDetectorResults` | Used by Slalom task |
| `/pontus/camera_<id>/camera_info` | Topic | `sensor_msgs/CameraInfo` | Intrinsics for gate/slalom geometry |
| `/auto_enable` | Topic | `std_msgs/Bool` | Hardware autonomy switch input |

#### Published Topics
| Name | Type | Message | Purpose |
| --- | --- | --- | --- |
| `/autonomy_mode` | Topic | `std_msgs/Bool` | Autonomy mode state (mirrors switch) |
| `/run_waypoints_markers` | Topic | `visualization_msgs/MarkerArray` | Planned waypoint visualization |
| `/run_waypoints_path` | Topic | `nav_msgs/Path` | Planned path visualization |

#### Services
| Name | Type | Message/Service | Purpose |
| --- | --- | --- | --- |
| `/pontus/get_gate_detection` | Service | `pontus_msgs/GetGateLocation` | Query current gate estimate (Gate task) |
| `/pontus/gate_side_information` | Service | `pontus_msgs/GateSideInformation` | Record/query which side of the gate was passed |
| `/pontus/add_semantic_object` | Service | `pontus_msgs/AddSemanticObject` | Seed semantic map with known objects |
| `/pontus/slalom_detector/set_enabled` | Service | `std_srvs/SetBool` | Toggle the Slalom detector on/off |

#### Actions
| Name | Type | Action | Purpose |
| --- | --- | --- | --- |
| `/pontus/go_to_pose` | Action | `pontus_msgs/GoToPose` | Drive to a commanded waypoint (used by all tasks) |
| `/pontus/search_region` | Action | `pontus_msgs/SearchRegion` | Explore a polygonal region (used by SearchRegionClient) |

---

### Key Tasks

#### Gate Task (`tasks/localization/gate_task.py`)
- **Subscribed:** `/pontus/odometry`, `/pontus/camera_2/yolo_results`, `/pontus/camera_2/camera_info`
- **Services used:** `/pontus/get_gate_detection`, `/pontus/gate_side_information`
- **Action used:** `/pontus/go_to_pose`
- **Purpose:** Search for, align to, and pass through the starting gate. Records which side the robot traversed.

#### Slalom Task (`tasks/slalom_task.py`)
- **Subscribed:** `/pontus/odometry`, `/pontus/slalom_detector/results`, `/pontus/camera_2/camera_info`
- **Service used:** `/pontus/slalom_detector/set_enabled` (to toggle detection)
- **Action used:** `/pontus/go_to_pose`
- **Purpose:** Detect red/white pole pairs, navigate between them in sequence.

#### Submerge (`tasks/localization/submerge.py`)
- **Action used:** `/pontus/go_to_pose`
- **Purpose:** Initial descent to a safe operating depth before starting tasks.

#### Waypoint Controller (`tasks/localization/waypoint_controller.py`)
- **Subscribed:** `/pontus/odometry`, `/auto_enable`
- **Action used:** `/pontus/go_to_pose`
- **Purpose:** Drive through a predefined list of waypoints from `run_info.py`.

#### Wait for Enable/Disable (`tasks/localization/wait_for_enable.py`, `wait_for_disable.py`)
- **Subscribed:** `/auto_enable`
- **Published:** `/autonomy_mode`
- **Purpose:** Gate tasks on the hardware autonomy switch.

---

### Runs

High-level run scripts sequence tasks for different competition phases.

- **`slalom_run.py`** --- Submerge, enable slalom detector, run SlalomTask.
- **`prequalification_run_localization.py`** --- Wait for enable → launch bringup → submerge → Waypoint controller (prequal mission).
- **`semi_run_localization.py`** --- Submerge -> GateTask.
- **`waypoint_run.py` / `waypoint_run_sim.py`** --- General waypoint missions with autonomy switch handling.

Each run inherits from `BaseRun`, which spins tasks sequentially and collects success/failure.

---

### Helpers

- **GoToPoseClient** (`helpers/GoToPoseClient.py`) — wraps `/pontus/go_to_pose` action for task code.
- **SearchRegionClient** (`helpers/SearchRegionClient.py`) — wraps `/pontus/search_region` action for region exploration.
- **SemanticMapSeeder** (`helpers/semantic_map_seeder.py`) — seeds semantic map via `/pontus/add_semantic_object`.
- **Run Info (`helpers/run_info.py`)** — defines static waypoints and semantic object coordinates (gate/slalom positions).
- **Waypoint Path Publisher (`helpers/run_info_waypoint_path_publisher.py`)** — publishes `/run_waypoints_markers` and `/run_waypoints_path` for visualization.

---

### Launch Files

#### `run_info_visualizer.launch.py`
- This is a tool that was used at comp to accuratly find where obstacles were before the runs by overlaying an image of the pool from google earth with a proper real world scaling and visualizing where objects and our path is. 
- Starts `nav2_map_server` with a pool map image.
- Includes `pontus_mapping/launch/mapping.launch.py`.
- Seeds semantic map (`semantic_map_seeder`).
- Runs waypoint path publisher (`run_info_waypoint_visualizer`).
- Publishes `/run_waypoints_markers` and `/run_waypoints_path`.

---

### Quick Topic/Service Summary

| Interface | Type | Purpose |
| --- | --- | --- |
| `/pontus/odometry` | Topic (`nav_msgs/Odometry`) | Pose feedback for all tasks |
| `/pontus/camera_*/yolo_results` | Topic (`pontus_msgs/YOLOResultArray`) | Gate/marker perception |
| `/pontus/slalom_detector/results` | Topic (`pontus_msgs/SlalomDetectorResults`) | Slalom perception |
| `/auto_enable` | Topic (`std_msgs/Bool`) | Autonomy switch input |
| `/autonomy_mode` | Topic (`std_msgs/Bool`) | Output autonomy switch state |
| `/run_waypoints_markers` | Topic (`visualization_msgs/MarkerArray`) | RViz/Foxglove waypoint viz |
| `/run_waypoints_path` | Topic (`nav_msgs/Path`) | RViz/Foxglove path viz |
| `/pontus/go_to_pose` | Action (`pontus_msgs/GoToPose`) | Navigation primitive |
| `/pontus/search_region` | Action (`pontus_msgs/SearchRegion`) | Region exploration primitive |
| `/pontus/get_gate_detection` | Service (`pontus_msgs/GetGateLocation`) | Query gate position |
| `/pontus/gate_side_information` | Service (`pontus_msgs/GateSideInformation`) | Record gate side traversal |
| `/pontus/add_semantic_object` | Service (`pontus_msgs/AddSemanticObject`) | Seed semantic map |
| `/pontus/slalom_detector/set_enabled` | Service (`std_srvs/SetBool`) | Toggle slalom detector |


---

## `pontus_sim`

### Overview
Gazebo simulation environments, models, and topic bridges. Exposes command and thruster topics used by controllers for testing in simulation.

### Subscribed Topics / Services
| Name | Type | Message/Service | Purpose |
| --- | --- | --- | --- |
| TODO | --- | --- | --- |

### Published Topics / Services
| Name | Type | Message/Service | Purpose |
| --- | --- | --- | --- |
| TODO | --- | --- | --- |

---
---

## `pontus_msgs`

### Overview
Custom message, service, and action definitions shared across the Pontus stack. These types connect perception -> mapping -> autonomy -> control.

---

## Messages

### `YOLOResult.msg`

---

### `YOLOResultArray.msg`

---

### `YOLOResultArrayPose.msg`

---

### `SlalomDetectionResults.msg`
---

### `Blueviewping.msg`

---

## Services

### `AddSemanticObject.srv`

---

### `GateSideInformation.srv`

---

### `GetGateLocation.srv`

---

### `GetVerticalMarkerLocation.srv`
---

## Actions

### `GoToPose.action`

---

### `SearchRegion.action`

---

## Quick Reference Table

| Type | Name | Purpose |
| --- | --- | --- |
| **msg** | `YOLOResult` | Single YOLO detection (class, score, bbox) |
| **msg** | `YOLOResultArray` | Batch of YOLO detections for a frame |
| **msg** | `YOLOResultArrayPose` | Detections + associated 3D rays/poses |
| **msg** | `SlalomDetectionResults` | Slalom red/white pole detections |
| **msg** | `Blueviewping` | Forward-looking sonar ping data |
| **srv** | `AddSemanticObject` | Add an object to the semantic map |
| **srv** | `GateSideInformation` | Set/get gate side traversal |
| **srv** | `GetGateLocation` | Query gate pole poses |
| **srv** | `GetVerticalMarkerLocation` | Query vertical marker pose |
| **action** | `GoToPose` | Navigate to a pose with feedback/result |
| **action** | `SearchRegion` | Explore a polygonal area |


---

## System Diagram
TODO: Create a diagram showing how data flows through the stack

## Last Updated: 9/9/2025 by Mitchell Turton