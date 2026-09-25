# WATonomous ASD admission assignment

A ROS 2 navigation pipeline for the [WATonomous ASD admission assignment](https://wiki.watonomous.ca/admission_assignments/asd_admission_assignment/). In the supplied Gazebo simulation, a differential-drive robot uses lidar and simulator odometry to build a map, plan around static obstacles, and drive to a point selected in Foxglove.

This repository is based on the [WATonomous training scaffold](https://github.com/WATonomous/wato_asd_training). My work is the C++ costmap, map memory, A* planner, and Pure Pursuit controller, their configuration and focused algorithm tests, and the Foxglove demo layout. The scaffold supplies Gazebo, Docker orchestration, and the odometry spoofer. AI tools assisted development and debugging; the checks below document what has been verified.

## Navigation pipeline

```text
/lidar → costmap → /costmap → map_memory → /map → planner → /path → control → /cmd_vel
                                 ↑                      ↑                  ↑
                           /odom/filtered         /goal_point       /odom/filtered
```

| Node | Role |
| --- | --- |
| `costmap` | Ray traces laser scans into an unknown/free/occupied local grid and adds graded obstacle costs. |
| `map_memory` | Aligns local grids with timestamp-matched odometry and maintains a global grid in `sim_world`. |
| `planner` | Runs eight-neighbor A* with footprint clearance, corner-cutting prevention, and costs for unknown and near-obstacle cells. It replans while a goal is active. |
| `control` | Tracks the route with Pure Pursuit and publishes bounded velocity commands. It stops on stale inputs, an empty route, goal arrival, or a close lidar obstacle. |

The assignment provides the robot pose through `/odom/filtered`; this project does not implement localization or SLAM. Unknown space is traversable with a penalty, so the route may change as the robot sees more of the environment.

## Run the simulation

Install Docker Engine or Docker Desktop, then create the ignored file `watod-config.local.sh` in the repository root:

```sh
ACTIVE_MODULES="robot gazebo vis_tools"
PLATFORM="amd64"
```

`amd64` uses the supplied ROS Humble image, including under emulation on Apple Silicon. From the repository root:

```sh
./watod build robot
./watod up -d
./watod ps
```

Connect Foxglove to the WebSocket URL printed by the running visualization container (on the tested Mac, `ws://127.0.0.1:10020`), then import [`config/autonomous_demo.json`](config/autonomous_demo.json). Publish a `geometry_msgs/msg/PointStamped` goal on `/goal_point` in the `sim_world` frame. Choose a point clear of obstacles by at least the configured robot radius. The dashboard shows the global map, path, robot pose, camera, and ROS logs.

For a command-line example, first get your container name from `./watod ps`; replace `<robot-container>` below:

```sh
docker exec <robot-container> bash -lc 'source /opt/watonomous/setup.bash && ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped "{header: {frame_id: sim_world}, point: {x: -6.0, y: 2.0, z: 0.0}}"'
```

## Verification and evidence

The standalone A* and Pure Pursuit tests can run without ROS 2:

```sh
c++ -std=c++17 -O2 -I src/robot/planner/include src/robot/planner/test/grid_search_test.cpp -o /tmp/planner_grid_test && /tmp/planner_grid_test
c++ -std=c++17 -O2 -I src/robot/control/include src/robot/control/test/pure_pursuit_test.cpp -o /tmp/control_pursuit_test && /tmp/control_pursuit_test
```

On September 25, 2026, both tests passed. The Docker robot container launched all four navigation nodes and the odometry spoofer, merged lidar observations into `/map`, and reported goal completion at `(6, -2)` after the endpoint correction. A later odometry sample was `(5.79, -2.11)`, within the planner's 0.3 m tolerance. An earlier short goal at `(-6, 2)` also completed. See [SUBMISSION.md](SUBMISSION.md) for the test history, limitations, and remaining video proof.

These checks support the demonstrated routes; they do not establish reliability across many goals or environments. An uninterrupted obstacle-avoidance recording is still needed for submission.
