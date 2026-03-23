#!/bin/bash
# Monitor RViz lag and ROS2 system performance

# Show CPU and memory usage
htop &

# Monitor ROS2 topic rates
ros2 topic hz /scan &
ros2 topic hz /tf &
ros2 topic hz /map &

# Show ROS2 node graph
rqt_graph &

# Show runtime monitor (if installed)
rqt_runtime_monitor &

# Print instructions
cat <<EOF

Monitor windows:
- htop: CPU/memory usage
- ros2 topic hz: topic rates (look for drops or slow rates)
- rqt_graph: node connections
- rqt_runtime_monitor: node health (if available)

Close windows when done. If you see high CPU, slow topic rates, or dropped messages, consider reducing scan/map update rates or running RViz on another machine.
EOF
