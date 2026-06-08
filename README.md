# MAVS Scene Builder Branch

![MAVS logo](docs/screenshots/CAVS_OPA_Horizontal.png)

This branch adds a browser-based scene and simulation editor to the
[MSU Autonomous Vehicle Simulator (MAVS)](https://github.com/Mississippi-State-University-OTM/MAVS).

The scene builder provides a visual workflow for assembling MAVS scenes from
existing assets, placing vehicle presets and sensors, creating waypoint paths,
previewing sensor coverage, and exporting simulation JSON files.

The editor runs locally and uses the assets already available in the repository's
`data` directory. It does not require a frontend build step.

## Quick Start

From the repository root, start the scene-builder server:

```powershell
python utilities\scene_builder\server.py
```

Open the editor in a browser:

```text
http://127.0.0.1:8765
```

The server provides the web interface, scans MAVS assets and sensor definitions,
opens native file dialogs, saves scene files, and launches Python-based previews.

## Requirements

The basic editor requires:

- Python 3
- A modern browser with WebGL support
- MAVS `.obj` assets under `data`

Additional functionality requires:

- Tkinter for native Load, Save As, Export Simulation, and Load Simulation dialogs
- A working MAVS Python installation for `Preview` and `Run Sim`
- A built `mavs_simulation.exe` when running exported simulation files through
  the native MAVS executable

## Major Features

### Scene Asset Browser

The left sidebar automatically scans `data` for `.obj` assets.

- Browse assets grouped by folder
- Search asset names and paths
- View generated asset thumbnails
- Drag assets into the viewport
- Load large existing MAVS scenes
- Adjust the blank scene ground-plane size

### Object Placement and Editing

Scene objects can be selected and transformed directly in the viewport or through
the inspector.

- Move, rotate, and scale objects with axis gizmos
- Edit position, yaw, pitch, roll, and scale numerically
- Select one object or marquee-select multiple objects
- Group and ungroup objects
- Duplicate objects and groups
- Snap objects to the highest mesh surface beneath them
- Delete selected objects
- See selection bounds and group bounds

Transform shortcuts:

| Action | Shortcut |
| --- | --- |
| Move mode | `W` |
| Rotate mode | `E` |
| Scale mode | `R` |
| Copy selection | `Ctrl+C` |
| Paste selection | `Ctrl+V` |
| Delete selection | `Delete` |
| Focus camera on selection | `F` |
| Toggle selected vehicle LiDAR preview | `V` |
| Duplicate while moving | Hold `Alt` while dragging a move gizmo |

### Camera Navigation

The editor camera supports both precise work and traversal across large scenes.

- Right-drag to freely orbit above or below the horizon
- Hold right mouse and use `W`, `A`, `S`, and `D` to move
- Shift-drag or middle-drag to pan
- Use the mouse wheel to zoom
- Focus the camera on a selection
- Adjust WASD movement speed from `0.1x` to `10x`

The camera movement-speed setting is saved in the browser draft.

### Random Object and Vegetation Zones

Random zones export to MAVS `Random` object blocks.

- Draw rectangular zones in the viewport
- Choose any available object or vegetation mesh
- Place objects using density or a fixed count
- Configure minimum spacing
- Configure minimum and maximum scale
- Configure Z offset and zone bounds

### Waypoint Paths

Multiple paths can be created and displayed in different colors.

- Create and name paths
- Place and move waypoint nodes
- Remove the most recent waypoint
- Save waypoint JSON files
- Assign vehicle presets to paths
- Show assigned vehicles in the path inspector

Waypoint files are saved under:

```text
data/waypoints
```

### Vehicle Presets and Controllers

Vehicle presets are loaded from MAVS RP3D vehicle definitions in:

```text
data/vehicles/rp3d_vehicles
```

Placed vehicles are represented as groups containing their chassis and tires.
Supported controller modes include:

- No Controller
- Human Controller
- Path Following

Path-following vehicles expose controls for:

- Desired speed
- Generated path-point spacing
- Steering scale
- Maximum steering angle
- Minimum and maximum lookahead distance

### Vehicle Sensors

Sensors can be attached to vehicle presets and positioned relative to the vehicle
center of gravity.

Supported sensor types include:

- LiDAR
- Camera
- GPS
- Compass
- Fisheye camera
- Radar
- IMU

Each sensor can define:

- Name and type
- Built-in MAVS model or JSON sensor definition
- X, Y, and Z offset
- Yaw, pitch, and roll
- Update rate

Sensor marker objects can be moved in the scene or snapped to the vehicle mesh.

### Dynamic Sensor Catalog

The scene builder automatically discovers sensor JSON files under:

```text
data/sensors
```

The catalog refreshes while the editor is open, so newly added or removed sensor
JSON files appear without restarting the editor.

The builder reads available sensor metadata to improve previews:

- Camera focal length and focal-plane dimensions define the displayed FOV
- LiDAR horizontal and vertical scan patterns define point-cloud sampling
- LiDAR minimum and maximum ranges are read when available
- Radar FOV and maximum range metadata are cataloged

Built-in sensor names are limited to models supported by the native MAVS
simulation loader. JSON-defined sensors export using `Input File`.

### Sensor Visualization

Sensor visualization is intended to help place sensors and identify coverage gaps
before running a simulation.

- Display camera field-of-view frustums
- Display LiDAR point-cloud returns
- Ray cast LiDAR points against vehicle and scene geometry
- Adjust LiDAR visualization range and point size
- Move sensors and immediately inspect changed coverage

The point-cloud preview is an editor visualization. The final MAVS simulation
still uses the native MAVS sensor implementation.

### File Menu

The File menu provides:

- New Scene
- Load Scene
- Save Scene
- Save Scene As
- Export Simulation
- Load Simulation

Load and Save As operations use native file dialogs.

## Files Produced by the Builder

The builder works with several MAVS JSON formats:

| Output | Default location | Purpose |
| --- | --- | --- |
| Scene JSON | `data/scenes` | Mesh instances, surface mesh, and random zones |
| Waypoint JSON | `data/waypoints` | Vehicle path points |
| Simulation JSON | `data/sims` | Native MAVS simulation configuration |
| Runtime vehicle JSON | `data/vehicles/rp3d_vehicles/_scene_builder_runtime` | Vehicle definition patched for scene compatibility when needed |

The browser also stores a local draft so work can be restored after refreshing
the page.

## Typical Workflows

### Example: Build and Preview a Scene

1. Start the local server.
2. Drag meshes from the asset browser into the viewport.
3. Use the transform gizmos or inspector to position objects.
4. Add random vegetation zones where needed.
5. Choose `File > Save Scene As`.
6. Click `Preview` to launch a MAVS camera preview.

### Example: Create a Path-Following Vehicle Simulation

1. Expand `Vehicle Presets` and place a vehicle.
2. Click `Add Path`.
3. Select the path and click `Place Waypoints`.
4. Place at least two waypoint nodes in the viewport.
5. Select the vehicle and choose `Path Following`.
6. Assign the path and adjust controller parameters.
7. Add and position sensors on the vehicle.
8. Choose `File > Export Simulation`.

The exported simulation can be run using the native executable:

```powershell
build\src\simulation\Release\mavs_simulation.exe data\sims\your_scene_sim.json
```

Or, depending on the local MAVS installation:

```powershell
install\bin\mavs_simulation.exe data\sims\your_scene_sim.json
```

### Example: Inspect LiDAR Blind Spots

1. Place a vehicle preset.
2. Add a LiDAR sensor or select an existing one.
3. Position the sensor using its offset fields or scene marker.
4. Enable `Show point cloud preview`.
5. Place nearby scene objects around the vehicle.
6. Move or rotate the sensor and compare the visible point returns.

### Example: Add a Custom JSON Sensor

Place a MAVS-compatible sensor definition in `data/sensors`, for example:

```json
{
  "Type": "lidar",
  "Min Range": 0.5,
  "Max Range": 80.0,
  "Scan Pattern": {
    "Horizontal Range": [-180.0, 180.0],
    "Horizontal Step": 1.0,
    "Vertical Range": [-15.0, 15.0],
    "Vertical Step": 2.0
  }
}
```

Within a few seconds, the sensor appears in the vehicle sensor-model list. The
builder uses its scan pattern for the point-cloud preview and exports the
definition as a sensor `Input File`.

## Native MAVS Compatibility

The scene builder exports simulation sensor blocks conservatively so generated
files remain compatible with the existing native MAVS loader.

- Supported native built-in sensors export using `Model`
- JSON-defined sensors export using `Input File`
- Compass sensors receive the legacy `Input File` field expected by MAVS
- Unsupported or incomplete sensor records are not written into native
  simulation configurations

After changing a scene, vehicle, path, or sensor configuration, export the
simulation again before running `mavs_simulation.exe`.

## Python Preview and Native Simulation

The two run paths serve different purposes:

- `Preview` launches a camera-focused Python preview of the current scene.
- `Run Sim` launches the scene builder's Python simulation workflow.
- `Export Simulation` creates a JSON configuration intended for the native
  `mavs_simulation.exe` application.

The Python runner is useful for rapid iteration from the editor. Native export
compatibility is retained so scenes and simulations can be shared with other MAVS
users.

## Current Scope and Limitations

- Native simulation export uses the first path-following vehicle that has at least
  two waypoints.
- Native simulation JSON currently stores resolved file paths for the scene,
  vehicle, and waypoint files. When sharing an export with a user who stores MAVS
  in a different location, update those paths or re-export on their machine.
- Dynamic moving actors are not yet editable through the scene-builder UI.
- Sensor visualization is an interactive placement aid rather than a replacement
  for running the native MAVS sensor simulation.
- LiDAR preview performance depends on scene mesh density and visualization range.

## Scene Builder Architecture

The scene builder is implemented as browser-native JavaScript modules with a
small Python HTTP server.

```text
utilities/scene_builder/
  app.js                    application initialization
  server.py                 local API, file dialogs, catalog scanning, export
  core/                     shared state, DOM references, lifecycle events
  interaction/              viewport input, picking, snapping, selection
  rendering/                WebGL renderer, camera, models, scene drawing
  scene/                    objects, selection, serialization, draft storage
  vehicles/                 vehicles, sensors, sensor UI and visualization
  paths/                    waypoint state and path UI
  zones/                    random-zone state and UI
  ui/                       inspector, asset browser, editor actions
```

The server rescans assets and sensor definitions from disk instead of requiring a
generated frontend asset manifest.

## Troubleshooting

### The Browser Shows JSON or Asset Loading Errors

Open the editor through the Python server rather than opening `index.html`
directly:

```powershell
python utilities\scene_builder\server.py
```

Then visit `http://127.0.0.1:8765`.

### The Native Simulation Exits Immediately

- Re-export the simulation from the current editor version.
- Confirm all referenced scene, waypoint, vehicle, and sensor files exist.
- Confirm the vehicle has a path containing at least two waypoints.
- Confirm custom sensors use valid MAVS sensor JSON files.

### Large Scenes Are Difficult to Navigate

Increase `Camera Move Speed` in the left Scene Tools panel. Reduce it again when
placing or aligning objects precisely.

### The LiDAR Preview Takes Time to Update

LiDAR visualization ray casts against scene geometry in a worker thread. Dense
meshes and large visualization ranges require more processing.

## About MAVS

MAVS is developed at Mississippi State University and provides:

- Real-time autonomous vehicle simulation
- Physics-based camera, LiDAR, GPS, radar, and IMU simulation
- Vehicle dynamics and path-following tools
- Realistic digital terrain and environmental effects
- C++, Python, MATLAB, ROS, and ROS 2 integration options

Full MAVS documentation is available at:

https://mississippi-state-university-otm.github.io/MAVS/

## Building MAVS

See the upstream
[MAVS build instructions](https://mississippi-state-university-otm.github.io/MAVS/docs/MavsBuildInstructions.html).

## License

MAVS is licensed under the
[MIT License](https://github.com/Mississippi-State-University-OTM/MAVS?tab=MIT-1-ov-file#readme).

## Citing MAVS

If you use MAVS for research, please cite one or more of the following:

- [Hudson, C., Goodin, C., Miller, Z., Wheeler, W., & Carruth, D. (2020). Mississippi State University Autonomous Vehicle Simulation Library.](http://gvsets.ndia-mich.org/documents/MS2/2020/MS2_1130_Mississippi%20State%20University%20Autonomous%20Vehicle%20Simulation%20Library_Paper.pdf)
- [Goodin, C., Carruth, D. W., Dabbiru, L., Hudson, C. H., Cagle, L. D., Scherrer, N., et al. (2022). Simulation-based testing of autonomous ground vehicles.](https://www.spiedigitallibrary.org/conference-proceedings-of-spie/12115/0000/Simulation-based-testing-of-autonomous-ground-vehicles/10.1117/12.2620502.short)
- [Carruth, D. W., Goodin, C., Dabbiru, L., Scherrer, N., Moore, M. N., Hudson, C. H., et al. (2024). Comparing real and simulated performance for an off-road autonomous ground vehicle in obstacle avoidance.](https://onlinelibrary.wiley.com/doi/pdf/10.1002/rob.22289)
