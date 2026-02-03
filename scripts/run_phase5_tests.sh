#!/bin/bash
# Phase 5 Verification Test Runner
# Run this in a separate terminal while exploration is active

echo "╔════════════════════════════════════════════════════════════╗"
echo "║           PHASE 5 VERIFICATION TEST RUNNER                ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "This script will monitor your exploration and track metrics:"
echo "  📊 Frontier detection accuracy"
echo "  🚀 Navigation success rate"
echo "  🗺️  Map coverage percentage"
echo "  ⏱️  Exploration time"
echo ""
echo "Make sure your robot is already running with exploration:=true"
echo ""
read -p "Press Enter to start Phase 5 metrics tracking..."

# Source the workspace
cd /home/pi/FEA_SLAM_WS
source install/setup.bash

echo ""
echo "✅ Starting Phase 5 metrics tracker..."
echo "📊 Updates every 10 seconds"
echo "🛑 Press Ctrl+C when exploration is complete to see final report"
echo ""
echo "─────────────────────────────────────────────────────────────"
echo ""

# Run the metrics tracker
ros2 run localization phase5_metrics

echo ""
echo "✅ Phase 5 metrics collection complete!"
