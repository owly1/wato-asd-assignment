# ASD assignment implementation

## Status

Latest result: the robot traversed around the central cylinder toward (6, -2), and a later goal command was reported reached after a path-endpoint fix. The public GitHub repository is available. Remaining deliverables are an uninterrupted demo video and the final submission to the required Discord channel.

Implemented the four navigation components in C++ using ROS 2. In a user-run Docker simulation on macOS, the robot planned a route to (-6, 2), followed it, reported goal reached, and stopped. Sending the same goal again correctly reported that it was already reached.

The short run confirms basic goal following and stopping. In an initial longer run to (6, -2), the robot moved around part of the central cylinder, then stopped near (-3.32, -3.30). `/path` became empty and the planner reported no route. That attempt exposed a clearance issue.

A source change added a soft clearance cost over 0.8 m beyond the hard footprint boundary, so A* prefers more room for tracking around corners. It also distinguishes blocked-start, blocked-goal, and disconnected-route failures. The subsequent run crossed the obstacle region, as detailed below. No video is included in this repository yet.

## Data flow

`/lidar → costmap → /costmap → map_memory → /map → planner → /path → control → /cmd_vel`

Map memory, planning, and control also consume `/odom/filtered`. Goals arrive on `/goal_point` as `geometry_msgs/msg/PointStamped` in `sim_world`.

| Component | Implementation |
| --- | --- |
| Costmap | Ray traces lidar into a 200 × 200 grid at 0.1 m resolution, distinguishes unknown/free/occupied space, and adds obstacle costs. |
| Map memory | Transforms local observations using timestamp-matched odometry into a persistent 600 × 600 grid in `sim_world`. |
| Planner | Eight-neighbor A* with footprint clearance, no diagonal corner cutting, and penalties for unknown and costly cells. Replans while a goal is active. |
| Controller | Pure Pursuit with a 0.8 m lookahead, speed capped at 0.25 m/s, and rotation capped at 0.5 rad/s. Stops for stale inputs, missing routes, goal completion, or close lidar obstacles. |

## Run on this Mac

Docker Desktop must be running. Run commands from the repository folder in the Mac terminal (the prompt ending in `%`). `docker exec` below runs ROS commands inside the Linux container automatically.

The ignored local `watod-config.local.sh` uses:

```sh
ACTIVE_MODULES="robot gazebo vis_tools"
PLATFORM="amd64"
```

Build and start:

```sh
./watod build robot
./watod up -d
```

Send the short goal used in the successful run:

```sh
docker exec watod_terrywang-robot-1 bash -c 'source /opt/watonomous/setup.bash && timeout 20 ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped "{header: {frame_id: sim_world}, point: {x: -6.0, y: 2.0, z: 0.0}}"'
```

Read progress:

```sh
docker logs --tail 30 watod_terrywang-robot-1
```

The container name above is specific to this Mac; other users should use their robot container name from `docker ps`.

## Demonstration

Import `config/autonomous_demo.json` into Foxglove and connect to `ws://127.0.0.1:10020`. The layout shows the global map, planned path, lidar, robot/environment models, and camera. It omits teleoperation controls so manual velocity commands cannot accidentally compete with autonomous commands from the panel.

Foxglove connected successfully after enabling Chrome’s “Apps on device” permission for app.foxglove.dev. The live camera, transforms, and global map were observed. The 3D panel can publish a point goal; monitor ROS node logs in a terminal with `docker logs -f`.

Once connected, record the map, planned path, robot movement, and stopping at the goal. A useful additional demonstration would navigate around the central cylinder to a clear destination on its opposite side. Select a clear goal on the map; a goal at (-3, 0) lies on the central cylinder boundary and was rejected in an earlier run.

## Evidence from the successful run

User-provided runtime logs, September 25, 2026:

```text
[control_node]: Following planned route
[control_node]: Stopped: at route endpoint or route is invalid
[planner_node]: Goal reached; published an empty path to stop control
[control_node]: Stopped: waiting for a planned route
[planner_node]: Goal received: (-6.00, 2.00) in sim_world
[planner_node]: Goal reached; published an empty path to stop control
```

## Limitations and design choices

- Odometry comes from the assignment simulator; this is not a SLAM implementation.
- Unknown cells are permitted with an added planning penalty, so the planner may initially propose travel into unexplored areas and replan as lidar reveals obstacles.
- The 1.5 m footprint radius is conservative and may reject narrow passages.
- A* runs on a discrete grid; Pure Pursuit follows the resulting waypoints.
- Close-obstacle stopping is reactive; robust operation across many routes has not yet been demonstrated.
- The launch/build environment uses AMD64 containers under emulation on this Mac.

## Submission checklist

- [x] Implement costmap, map memory, planner, and controller.
- [x] Observe a successful short goal-following run and stop.
- [x] Observe longer obstacle traversal and verify goal completion after endpoint correction.
- [ ] Record a video showing the navigation pipeline.
- [x] Publish the source in the applicant's GitHub repository.
- [ ] Submit the repository and video links through the required assignment channel.

AI assistance was used during development and troubleshooting. The applicant should be able to explain the algorithms, parameters, and limitations above.

## Follow-up run after clearance improvement

After rebuilding and resetting the simulation, the robot traversed around the central cylinder toward (6, -2). Observed odometry progressed to (4.685, -2.408), then (5.7405, -2.1553). This demonstrated longer obstacle traversal, but the final position was approximately 0.302 m from the requested point, just outside the planner's 0.3 m goal tolerance.

A final source correction makes the path end at the exact requested goal rather than its grid-cell center, eliminating the mismatch between controller and planner stopping references. The endpoint correction was rebuilt and verified live. On resuming the goal, the controller followed the route and stopped. Republishing (6, -2) at 09:07:58 EDT produced: `Goal reached; published an empty path to stop control`. A later odometry sample was (5.7919, -2.1083), approximately 0.235 m from the goal. The obstacle traversal and final stopping were verified across these runs; a single uninterrupted recorded run remains to be captured.
