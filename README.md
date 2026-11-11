# FFW-SG2 Sim Control Panel

A one-window GUI for **SLAM mapping** and **saved-map navigation (Nav2 + AMCL)** of the ROBOTIS FFW-SG2 in Gazebo.

This repo contains only the GUI and its launcher. No ROBOTIS code is included; get ai_worker from the official repository: **https://github.com/ROBOTIS-GIT/ai_worker** (branch `jazzy`).

**Project page and videos:** see `index.html` (published via GitHub Pages).

## Files

- `sim_control_panel.py`: the GUI
- `run_nav_sim.sh`: launcher the GUI calls

## Quick start

```bash
# 1. Official ai_worker
git clone -b jazzy https://github.com/ROBOTIS-GIT/ai_worker.git
cp sim_control_panel.py run_nav_sim.sh ai_worker/
cd ai_worker && ./docker/container.sh start && ./docker/container.sh enter

# 2. Inside the container (once)
apt-get update && apt-get install -y python3-tk

# 3. Run
export GZ_IP=127.0.0.1
python3 /root/ros2_ws/src/ai_worker/sim_control_panel.py
```

**Map:** Start mapping (SLAM) → drive with W/S, A/D, Q/E → set a name in **Save to** → Save Map.
**Navigate:** pick the map → Start saved-map nav → RViz **2D Pose Estimate** → **2D Goal Pose** on a white area.

Tested with ai_worker 1.3.2 (`jazzy`).

## Disclaimer

Independent project, not affiliated with ROBOTIS. FFW-SG2 and ai_worker belong to ROBOTIS Co., Ltd. under their own license.

## License

Apache-2.0
