#!/usr/bin/env bash
set -euo pipefail

# Simple preview helper:
# - regenerates URDF from XACRO
# - writes a temporary params file for robot_state_publisher
# - restarts robot_state_publisher with the new URDF
# Usage: ./scripts/preview_robot.sh [--xacro PATH] [--urdf PATH]

XACRO=${1:-/home/pi/FEA_SLAM_WS/src/fea_slam/description/robot_core.xacro}
URDF=${2:-/tmp/robot_core.urdf}
PARAMS=${3:-/tmp/rsp_params.yaml}

echo "Generating URDF from: $XACRO -> $URDF"
ros2 run xacro xacro "$XACRO" -o "$URDF"

echo "Creating params file: $PARAMS"
python3 - <<PY
from pathlib import Path
urdf = Path('$URDF').read_text()
params = 'robot_state_publisher:\n  ros__parameters:\n    robot_description: |\n'
for line in urdf.splitlines():
    params += '      ' + line + '\n'
Path('$PARAMS').write_text(params)
print('Wrote', '$PARAMS')
PY

echo "Stopping any existing robot_state_publisher started by this script..."
pkill -f "robot_state_publisher --ros-args --params-file $PARAMS" || true

echo "Starting robot_state_publisher with new URDF"
nohup ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$PARAMS" >/tmp/rsp.log 2>&1 &
sleep 0.5
echo "robot_state_publisher started (logs: /tmp/rsp.log)"
echo "Open RViz2 and add a 'RobotModel' display, set Fixed Frame to 'base_link' and it will show the robot." 
echo "Tip: Install 'entr' (sudo apt install entr) and run: while true; do ls $XACRO | entr -p $0; done to auto-reload on file save."
