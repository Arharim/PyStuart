# PyStuart - Stewart Platform Simulator

Interactive 3D simulator for a Stewart Platform (hexapod) with real-time
inverse kinematics. The v2 line grows the simulator into the full motion
rig stack: motion cueing (washout), sim racing telemetry and hardware
transport.

## Features

- Real-time 6-DOF platform control
- Inverse kinematics visualization
- Camera controls (rotate, pan, zoom)
- Servo angle feedback
- Reachability detection
- Trajectory animation (CSV import, playback, speed/loop)
- Pose presets (save/load/delete named configurations)
- Servo angle export (current pose and full trajectory) to CSV
- Workspace boundary analysis with IK reachability grid
- Cueing pipeline: AC UDP / CSV telemetry -> classical washout -> live 3D
- Hardware tab: serial link to the Bridge, CMD_ACT streaming, SRV_FB / PWR_MON readouts
- Headless runner for rig-side service duty: `pystuart-headless`

## Requirements

- Python 3.8+
- PyQt6
- PyOpenGL
- NumPy

## Installation

```bash
git clone https://github.com/Arharim/PyStuart.git
cd PyStuart

# editable install (recommended for development)
pip install -e .

# or just the dependencies
pip install -r requirements.txt
```

## Usage

### Run the application

```bash
python -m pystuart
# or, after pip install
stewart-platform
```

### GUI tabs

- **Manual** - platform sliders, presets, trajectory animation, workspace analysis
- **Cueing** - pick a telemetry source (Assetto Corsa UDP or CSV replay) and
  run it through the washout chain; the 3D view shows the resulting platform
  motion live
- **Hardware** - connect to the Bridge over serial, stream the current pose
  to the servos as calibrated CMD_ACT targets, watch SRV_FB / PWR_MON
  feedback; torque is released on disconnect and on window close

### Headless runner

```bash
python -m pystuart.apps.headless ac --serial COM7
# or, after pip install
pystuart-headless csv recording.csv --serial COM7
```

### Controls

**Platform Control:**
- Translation sliders (X, Y, Z): Move platform position
- Rotation sliders (Roll, Pitch, Yaw): Rotate platform orientation
- Reset Pose: Return platform to initial position
- Reset Camera: Reset camera view

**Camera Control:**
- Left mouse: Rotate camera around scene
- Right mouse: Pan camera
- Middle mouse: Vertical offset
- Mouse wheel: Zoom in/out

## Project Structure

```
PyStuart/
├── src/pystuart/
│   ├── core/        # IK, geometry, limits (pure numpy)
│   ├── cueing/      # washout filters, telemetry sources (planned)
│   ├── transport/   # USB CDC / BLE / UDP hardware links (planned)
│   ├── gui/         # PyQt6 widgets and OpenGL rendering
│   ├── apps/        # entry points (gui, headless)
│   └── __main__.py  # python -m pystuart
├── tests/           # pytest suite
├── samples/         # sample trajectories
├── docs/            # local working notes (not committed)
├── requirements.txt
└── pyproject.toml
```

## Technical Details

### Platform Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Base radius | 76 | mm |
| Platform radius | 60 | mm |
| Horn length | 40 | mm |
| Rod length | 130 | mm |
| Initial height | 120.28 | mm |

### Coordinate System

- X-axis: Red
- Y-axis: Green
- Z-axis: Blue (vertical)

## Running Tests

```bash
pip install pytest
python -m pytest
```

## License

MIT License

## Version

v2.0.0-dev
