# V2 YOLO/SONOPTIX Fusion Test Suite

This suite tests `image_to_cluster_coordinator.py` at two levels:

1. **Unit tests** exercise the assignment, planar tracking, bearing-sector association, stereo triangulation, evidence accumulation, promotion, and per-camera model separation.
2. **Integration testing** spins the real `ImageCoordinator` in-process with ROS publishers and a fake `AddSemanticObject` service. It does not require cameras, SONOPTIX hardware, recorded bags, or the semantic-map node.

The integration test deliberately uses asynchronous camera timestamps. It does **not** create a three-way message synchronizer.

## Files

Copy the provided `test/` directory and `pytest.ini` into the root of the ROS package containing the V2 fusion source:

```text
<your_package>/
├── package.xml
├── setup.py or CMakeLists.txt
├── pytest.ini
├── <python_package>/
│   └── image_to_cluster_coordinator.py
└── test/
    ├── __init__.py
    ├── conftest.py
    ├── fusion_test_helpers.py
    ├── test_fusion_algorithms.py
    └── test_fusion_node_integration.py
```

`conftest.py` normally discovers exactly one file named `image_to_cluster_coordinator.py` beneath the package root. If your filename or layout differs, set one of:

```bash
export PONTUS_FUSION_SOURCE="$PWD/src/path/to/image_to_cluster_coordinator.py"
```

or, after building and sourcing the workspace:

```bash
export PONTUS_FUSION_MODULE="your_python_package.image_to_cluster_coordinator"
```

Do not set both.

## Required test dependencies

Add these to `package.xml` if they are not already present:

```xml
<test_depend>ament_pytest</test_depend>
<test_depend>python3-pytest</test_depend>
```

The node's normal runtime dependencies must also be declared, including the packages that provide `rclpy`, `geometry_msgs`, `sensor_msgs`, `sensor_msgs_py`, `vision_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_sensor_msgs`, `image_geometry`, `pontus_msgs`, and `pontus_bringup`.

Then install dependencies from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Replace `jazzy` if the project uses another ROS 2 distribution.

## ament_python package setup

For a normal `ament_python` package, make sure `setup.py` includes:

```python
setup(
    # existing fields ...
    tests_require=["pytest"],
)
```

Pytest files under `test/` are discovered by the ROS/colcon test tooling.

## ament_cmake_python package setup

For a package using CMake to install Python, add this before `ament_package()` in `CMakeLists.txt`:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    fusion_v2_tests
    test
    TIMEOUT 120
  )
endif()
```

Also add:

```xml
<test_depend>ament_cmake_pytest</test_depend>
```

## Run directly with pytest

First source ROS and the built workspace so that project messages and Python packages are importable:

```bash
cd ~/your_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

From the package root, run the fast tests only:

```bash
pytest -q -m "not integration" test
```

Run the ROS integration test only:

```bash
pytest -q -m integration test
```

Run everything:

```bash
pytest -q test
```

To show ROS log output while diagnosing a failure:

```bash
pytest -s -vv -m integration test/test_fusion_node_integration.py
```

## Run through colcon

From the workspace root:

```bash
colcon build --symlink-install --packages-select <your_package>
source install/setup.bash
colcon test --packages-select <your_package> --event-handlers console_direct+
colcon test-result --verbose
```

The integration test starts its own fake semantic-object service and uses unique `/fusion_test/...` input topics. It still uses the production unlabeled output topic:

```text
/pontus/unlabled_candidate_tracks
```

Do not run a second instance of the production fusion node in the same ROS domain while executing this test.

For stronger isolation in local development or CI, assign a temporary domain:

```bash
export ROS_DOMAIN_ID=88
colcon test --packages-select <your_package> --event-handlers console_direct+
```

## What the tests cover

### Assignment and tracking

- Hungarian assignment obtains the global minimum rather than a greedy pairing.
- Invalid associations remain unmatched.
- SONOPTIX matching is one-to-one.
- Track association and state are independent of `z`.
- Tentative-track retirement follows configured missed-scan limits.
- Exact duplicate sonar timestamps are ignored.

### Camera fusion

- A track inside the bounding-box bearing sector is accepted.
- A track outside the sector plus class margin is rejected.
- Stale sonar tracks are rejected by camera association.
- Camera association does not use track `z` or camera-ray `z`.
- Two cameras own separate `PinholeCameraModel` instances and can use different intrinsics.

### Stereo and promotion

- Two valid planar bearings recover the expected intersection.
- Weak-parallax stereo is rejected.
- Camera evidence accumulates over time.
- A confirmed-unlabeled track promotes only after the configured evidence, confidence, and margin tests pass.

### End-to-end ROS behavior

The integration test performs this sequence:

1. Starts a fake `AddSemanticObject` service.
2. Starts the real fusion node with two test cameras.
3. Inserts static map-to-camera transforms into the real tf2 buffer.
4. Publishes different `CameraInfo` intrinsics for each camera.
5. Publishes a SONOPTIX cloud and republishes the exact duplicate.
6. Publishes three more sonar scans with the same `x,y` and radically different `z`.
7. Verifies one confirmed-unlabeled planar track and a one-point unlabeled `PointCloud2` snapshot.
8. Publishes camera-1 evidence without camera 2 and verifies it is processed immediately.
9. Publishes camera-2 evidence later, within the optional stereo time window.
10. Publishes another camera-1 observation and verifies semantic promotion.
11. Verifies the semantic service request contains the expected class and sonar `x,y` position.
12. Verifies the complete unlabeled snapshot becomes empty after promotion.

## Recommended competition workflow

Run the pure unit suite on every change:

```bash
pytest -q -m "not integration" test
```

Run the integration suite before merging and before building the competition image:

```bash
pytest -q -m integration test
```

After those pass, replay representative bags and inspect these runtime properties:

- track count remains stable under repeated sonar clusters;
- no duplicate-stamp warnings occur during normal live operation;
- camera-specific bearing markers align with visible objects;
- camera 1 and camera 2 can each add evidence independently;
- stereo bonuses occur only when both cameras associate to the same sonar track;
- unlabeled snapshots contain all and only confirmed-unlabeled tracks;
- semantic publication occurs once at promotion and then at the configured refresh period.

Bag replay is intentionally kept outside the automated suite because the exact topic names, TF tree, detector message schema, and bag availability are project-specific. Once a small known-good competition bag is available, it should become a third test layer with assertions on counts and final semantic positions.

## Likely failure causes

### The fusion source cannot be found

Set:

```bash
export PONTUS_FUSION_SOURCE=/absolute/path/to/image_to_cluster_coordinator.py
```

### `ModuleNotFoundError` for `pontus_msgs` or `pontus_bringup`

Build and source the workspace before running pytest:

```bash
colcon build --symlink-install
source install/setup.bash
```

### Integration test hangs while constructing `ImageCoordinator`

The production constructor waits for the semantic service. The test creates that service first, so a hang usually means DDS discovery is being blocked or another test has left the ROS context in a bad state. Run the integration test by itself with a fresh domain:

```bash
ROS_DOMAIN_ID=88 pytest -s -vv -m integration test
```

### TF lookup fails

Confirm the installed ROS version exposes `tf2_ros.Buffer.set_transform_static`. This is available in current ROS 2 tf2 Python implementations. If your distribution differs, replace the two direct buffer insertions in the integration test with a `StaticTransformBroadcaster`.

### Detection helper fails on `bbox.center`

The helper supports both common `vision_msgs/BoundingBox2D` layouts: a center with `.position.x/.position.y` and a center with `.x/.y`. If your custom message differs, modify only `make_detection()` in `fusion_test_helpers.py`.
