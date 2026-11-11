# FFW-SG2 Localization Sim Control Panel

A single-window GUI for running **SLAM mapping** and **saved-map navigation (Nav2 + AMCL)** for the ROBOTIS **FFW-SG2** in Gazebo simulation.

It is an add-on for the official ROBOTIS **ai_worker** repository. This project contains **only** the GUI and its launcher script. No ROBOTIS code is included here. You install ai_worker from the official source and then add these two files to it.

> Official ai_worker repository: **https://github.com/ROBOTIS-GIT/ai_worker**

![Sim Control Panel](docs/screenshot.png)

---

## Features

- **Start mapping (SLAM)**: launches Gazebo, the FFW-SG2, SLAM and RViz with one click.
- **Save / load / delete maps**: saves maps with `map_saver_cli`, picks a map from a dropdown, and deletes a map (`.yaml` + image) after confirmation.
- **Start saved-map nav**: relaunches with Nav2 + AMCL on the selected map.
- **Manual drive**: on-screen pad or keyboard (W/S, A/D, Q/E) publishing `Twist` to `/cmd_vel` for the swerve base. It stays silent when idle so it doesn't fight Nav2.
- **Stop all**: shuts down the whole stack cleanly.

## Repository contents

| File | Purpose |
|---|---|
| `sim_control_panel.py` | The GUI (Tkinter + rclpy) |
| `run_nav_sim.sh` | Launcher the GUI calls to start Gazebo, the controllers and Nav2 (SLAM or saved-map mode) |

## Requirements

- Ubuntu with Docker, Docker Compose and an NVIDIA GPU (as required by ai_worker)
- The official **ai_worker** repository, branch `jazzy`
- Tested with ai_worker **1.3.2** (`jazzy`). Newer releases may need small adjustments.

---

## Installation

### 1. Get the official ai_worker

Follow the instructions in the official repository:
**https://github.com/ROBOTIS-GIT/ai_worker**

In short:

```bash
git clone -b jazzy https://github.com/ROBOTIS-GIT/ai_worker.git
cd ai_worker
./docker/container.sh start
./docker/container.sh enter
```

Inside the container, check that the navigation packages are available:

```bash
ros2 pkg list | grep -E "ffw_navigation|ffw_bringup"
```

If nothing is listed, build the workspace:

```bash
cd /root/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

### 2. Add this project's files

Copy both files into the **root of your ai_worker clone** on the host:

```bash
cp sim_control_panel.py run_nav_sim.sh /path/to/ai_worker/
chmod +x /path/to/ai_worker/run_nav_sim.sh
```

Inside the container, the ai_worker folder is mounted at `/root/ros2_ws/src/ai_worker`, so the files appear there.

### 3. Install Tkinter (inside the container, once)

```bash
apt-get update && apt-get install -y python3-tk
```

---

## Usage

Inside the container:

```bash
export GZ_IP=127.0.0.1
python3 /root/ros2_ws/src/ai_worker/sim_control_panel.py
```

The panel starts the simulation for you, so don't run `run_nav_sim.sh` manually.

### Create and save a map

1. Press **Start mapping (SLAM)** and wait for Gazebo and RViz (about 30–60 s).
2. Click the panel window and drive with **W/S** (move), **A/D** (turn) and **Q/E** (strafe).
3. Watch RViz until the areas you need are mapped. White is free space, black is walls, grey is unexplored.
4. In **Save to:**, set a new name, e.g. `.../ffw_navigation/maps/room1`.
5. Press **Save Map** while SLAM is still running, then press **⟳**.

### Navigate on a saved map

1. Select the map in **Load map** and press **Start saved-map nav**.
2. In RViz, use **2D Pose Estimate** to set where the robot is.
3. Use **2D Goal Pose** on a **white (free)** area. The robot drives there.

### Delete a map

Select it in **Load map** and press **🗑 Delete**. A map in use by a running nav stack can't be deleted; press **Stop all** first.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| Gazebo doesn't load the world | Run `export GZ_IP=127.0.0.1` before starting the panel. |
| `sim_control_panel.py` not found inside the container | The container was started from a stale mount. Run `./docker/container.sh stop`, then `start` and `enter`. |
| Robot doesn't move after starting saved-map nav | Expected: set **2D Pose Estimate**, then **2D Goal Pose** in RViz. |
| `Failed to create plan` in the Nav2 log | The goal is outside the mapped area or blocked. Pick a white cell, or map that area first. |
| Something else fails | Check the logs in `/tmp/nav_sim_logs/`. |

Useful checks in a second container terminal:

```bash
ros2 topic hz /clock               # simulation running
ros2 control list_controllers      # swerve_drive_controller should be active
ros2 lifecycle get /bt_navigator   # should be active
```

---

## Disclaimer

This is an independent project and is **not affiliated with or endorsed by ROBOTIS**. FFW-SG2, AI Worker and the ai_worker software are the property of ROBOTIS Co., Ltd. and are distributed under their own license; see the official repository: https://github.com/ROBOTIS-GIT/ai_worker

## License

The files in this repository are released under the **Apache License 2.0**.
