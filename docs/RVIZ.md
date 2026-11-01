# The RViz view

`interceptor_ws/src/interceptor_drone/rviz/interceptor_config.rviz` is the
operator's view of the whole pipeline: Gazebo shows the world, RViz shows what the
drone *thinks* — what its sensors measured, what the tracker made of it, what the
planner asked for and what the safety filter let through.

It starts with every launch that wraps `interceptor_full.launch.py`
(`make full`, `follow.launch.py`, `intercept.launch.py`, `intercept_demo.launch.py`);
`rviz:=false` turns it off. On its own:

```bash
ros2 run rviz2 rviz2 -d install/interceptor_drone/share/interceptor_drone/rviz/interceptor_config.rviz
ros2 run interceptor_drone mission_viz_node.py
```

## mission_viz_node

Most of the pipeline talks in custom messages (`TargetDetection`,
`TargetStateArray`, `FlightSetpoint`, `MissionStatus`, `GuidanceCommand`) that
RViz cannot draw. `src/tools/mission_viz_node.py` subscribes to them and
republishes what they mean as markers and paths, one topic per layer. It only
watches: nothing in the pipeline reads its output, and the launch file starts it
only together with RViz.

| Topic | Type | What it draws |
|---|---|---|
| `/viz/perception` | `MarkerArray` | `camera_fov`: the left camera's frustum (60° HFOV, 4:3, 25 m), locked to its optical frame. `detections`: every `/target/detection_3d` of the last 2 s, a fading point coloured by class |
| `/viz/estimation` | `MarkerArray` | `covariance`: the 2σ position ellipsoid of every track (eigen-decomposition of the 3×3 covariance). `prediction`: the selected target's constant-acceleration path over the next 2 s, dashed |
| `/viz/guidance` | `MarkerArray` | `range`: the line to the selected target and its length. `planner_command`: `/setpoint/raw`'s velocity (1 s of travel). `capture_envelope`: the `capture_radius` sphere around the target (INTERCEPT). `altitude`: drop lines to the ground under both drones |
| `/viz/hud` | `MarkerArray` | Text above the interceptor: mode and arming, phase, target (id, class, speed, coasting), the guidance law, `N`, `V_c`, `t_go` and the zero-effort miss (INTERCEPT), speed, altitude, wind, and the safety filter's state and reason (orange when it intervenes, red on an abort) |
| `/viz/world` | `MarkerArray` (latched) | The geofence |
| `/viz/interceptor_path` | `Path` | The interceptor's trail from `/odom` |
| `/viz/target_estimate_path` | `Path` | The tracker's estimate of the selected target |
| `/viz/target_truth_path` | `Path` | The intruder's true trail from `/target_drone/odom` (the simulator's ground truth, never read by the pipeline) |

The geofence and the capture radius are read from `safety_params.yaml` and
`intercept_params.yaml`, so the drawing is the limit the filter and the guidance
enforce. The other parameters (`frustum_range`, `detection_memory`,
`prediction_horizon`, `ellipsoid_sigma`, `path_length`, `rate`) are declared by
the node with their defaults.

Markers have a 0.5 s lifetime and the node drops a target or a track list that has
not been refreshed for 1 s, so a layer whose source stops disappears instead of
freezing on the screen. A backwards jump of the sim clock starts the trails over.

## The displays

The tree follows the pipeline, one group per layer:

| Group | Displays |
|---|---|
| World | Ground grid (5 m cells, 200 m), world origin axes, geofence |
| Platform | The interceptor and the target drone models, the interceptor's body axes (FLU), its trail, the TF tree (off by default) |
| Perception | Camera FOV + raw detections, the left camera; the YOLO overlay and the SGBM depth (0-40 m) with `perception_source:=vision`; the right camera and the disparity (off by default) |
| Estimation (fusion) | The tracker's own markers (`/tracks/markers`: orange selected, green confirmed, grey tentative or coasting), the covariance ellipsoids and the prediction, the estimated trail (orange) against the true one (white): the gap between them is the tracking error |
| Guidance & Safety | Range + planner command, the PN vectors (`/guidance/vectors`), the keep-out circles and the safe setpoint (`/safety/keepouts`) |
| Mission HUD | The text above the interceptor |

**Reading the safety filter.** The translucent blue arrow is what the planner
asked for, the safety filter's arrow (green when unchanged, orange when modified)
is what the controller flies. When a keep-out circle is in the way the two arrows
split, and the HUD says why.

**Detection colours:** person blue, car yellow, truck orange, bicycle purple, uav
pink.

## Views

The default view orbits the interceptor (target frame `base_link`), so the drone
stays in the middle of the screen. The saved views (Views panel, double-click):

- **Chase Cam** — behind the interceptor, turning with its heading: the cameras'
  point of view.
- **Orbit Interceptor** — the default view.
- **Arena Overview** — the whole park from above the south-west corner.
- **Top Down** — orthographic map of the arena.

The image panels dock where Qt puts them on the first start. Arrange them once and
save with `Ctrl+S` from the source tree (`rviz/` is installed with
`--symlink-install`, so the saved layout lands in the repository).
