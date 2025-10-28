# SCMS_Luca

Utilities for running an image-based visual servo (IBVS) pick-and-place demo with a Dobot Magician. The scripts in `src/` talk to an Intel RealSense RGB-D camera, detect coloured blocks, compute a hand–eye calibration, and drive the robot via `rosbridge`.

## Project Layout
- `requirements.txt` – Python dependencies used by the scripts (install into a venv).
- `src/realsense.py` – Thin wrapper that exposes aligned RGB-D frames and cached intrinsics.
- `src/robot_io.py` – ROS WebSocket client for the Dobot course driver (`roslibpy`).
- `src/block_detection.py` – Interactive HSV + depth blocker; exports detections to JSON.
- `src/calibrate.py` – Collect correspondences and solve for `config/handeye.json`.
- `src/ibvs_control.py` – IBVS helpers (interaction matrix, adjoint transform).
- `src/main.py` – End-to-end pick routine that consumes the saved calibration/detections.

Run the scripts from the `SCMS_Luca` directory so relative paths like `config/` and `detections/` resolve correctly.

## Prerequisites
- Python 3.8+ on a machine that can see both the RealSense camera and the robot controller.
- Intel RealSense SDK (`librealsense2`) installed; `pyrealsense2` must match the SDK version.
- A running Dobot Magician ROS driver with `rosbridge_websocket` exposed (default `ws://10.42.0.1:9090`).
- ROS message packages (`geometry_msgs`, `sensor_msgs`, `std_msgs`) available in the Python environment.
- Network route from this machine to the robot’s Raspberry Pi (or update `host`/`port` in `robot_io.py`).

> Tip: `rospy` and the ROS message packages listed in `requirements.txt` are typically provided by the ROS desktop installation, not PyPI. Install them through your ROS distribution (e.g. `sudo apt install ros-noetic-rospy ros-noetic-geometry-msgs ...`). You’ll also need `roslibpy`, which can be installed with `pip install roslibpy`.

## Environment Setup
1. Create and activate a virtual environment (optional but recommended):
   ```bash
   cd SCMS_Luca
   python3 -m venv .venv
   source .venv/bin/activate
   ```
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   pip install roslibpy  # listed separately from requirements.txt
   ```
   Ensure the RealSense SDK is already installed so that `pyrealsense2` can be imported.

## Workflow

### 1. Hand–Eye Calibration (`calibrate.py`)
Generates the rigid transform between the camera and robot base.
```bash
python src/calibrate.py
```
- The script starts the RealSense and connects to the Dobot driver; ensure the robot safety state reaches `OPERATING(4)`.
- In the OpenCV window: move the tool tip to a visible feature, click it, then press `r` to record the pixel + pose pair. Collect at least three poses (≈5 is typical).
- `u` removes the last correspondence, `q` quits, and `s` solves for `T_rc` and saves `config/handeye.json`.

### 2. Capture Block Detections (`block_detection.py`)
Segments coloured blocks and exports their pixel/depth measurements.
```bash
python src/block_detection.py
```
- Cycles through colour masks with `c`; adjust depth band with `+`/`-`.
- Press `s` to write `detections/blocks_snapshot.json`. This file is consumed by `main.py`, so ensure the labels and depths look reasonable in the preview.
- `q` closes the viewer.

### 3. Execute the IBVS Pick Routine (`main.py`)
Runs the closed-loop pick-and-place using the saved calibration and detections.
```bash
python src/main.py
```
- Requires `config/handeye.json` (from calibration) and `detections/blocks_snapshot.json` (from detection) to exist.
- The script opens the RealSense stream, connects to `rosbridge`, hovers to a nominal pose, then iterates through each block JSON entry, servoing onto the block, gripping it, and dropping it in a colour-coded zone.
- Check the safety state on the Dobot controller before running. If the script reports that the driver is not `OPERATING(4)`, home/enable the robot on the Raspberry Pi and rerun.

## Troubleshooting
- **`pyrealsense2` import errors**: confirm the Intel RealSense SDK is installed and the camera is connected; reinstall the matching wheel (`pip install pyrealsense2==<SDK version>`).
- **ROS connection issues**: verify `rosbridge_websocket` is running on the robot host and reachable. Update `host`/`port` in `robot_io.py` or pass them into `DobotROSClient` if your network differs.
- **Missing ROS Python modules**: activate the ROS environment before starting the virtual environment (e.g. `source /opt/ros/noetic/setup.bash`) so that `rospy` and message packages resolve.
- **JSON/config not found**: ensure you run the scripts from the `SCMS_Luca` directory so relative paths point to the correct `config/` and `detections/` folders.

Once everything is calibrated and detections look clean, you can iterate quickly by rerunning `block_detection.py` to refresh targets or tweaking controller gains in `src/ibvs_control.py` and rerunning `main.py`.
