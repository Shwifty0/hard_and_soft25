#!/bin/bash

# This script checks your ROS 2 Jazzy and Gazebo Harmonium setup
# and helps diagnose issues with the RoboCar

echo "=== RoboCar Diagnostics ==="
echo ""

# Check ROS distro
echo "ROS distro: $ROS_DISTRO"
echo ""

# Check if required ROS packages are installed
echo "=== Checking for required ROS packages ==="
if dpkg -l | grep -q ros-$ROS_DISTRO-ros-gz-bridge; then
    echo "✓ ros-gz-bridge is installed"
else
    echo "✗ ros-gz-bridge is missing. Install with:"
    echo "  sudo apt install ros-$ROS_DISTRO-ros-gz-bridge"
fi

if dpkg -l | grep -q ros-$ROS_DISTRO-ros-gz-sim; then
    echo "✓ ros-gz-sim is installed"
else
    echo "✗ ros-gz-sim is missing. Install with:"
    echo "  sudo apt install ros-$ROS_DISTRO-ros-gz-sim"
fi
echo ""

# Check if Gazebo is installed and its version
echo "=== Checking Gazebo installation ==="
if command -v gz &> /dev/null; then
    echo "✓ Gazebo is installed"
    echo "Gazebo version: $(gz --version)"
else
    echo "✗ Gazebo command not found. Make sure Gazebo Harmonium is installed."
fi
echo ""

# Check if the RoboCar simulation package is properly built
echo "=== Checking RoboCar simulation package ==="
if ros2 pkg list | grep -q robocar_simulation; then
    echo "✓ robocar_simulation package is installed"
else
    echo "✗ robocar_simulation package not found. Build the package with:"
    echo "  cd ~/SOSE25/hard_soft/robocar_ws"
    echo "  colcon build --symlink-install"
    echo "  source install/setup.bash"
fi
echo ""

# Check for required URDF and launch files
echo "=== Checking required files ==="
WORKSPACE_DIR="$HOME/SOSE25/hard_soft/robocar_ws"
if [ -f "$WORKSPACE_DIR/src/robocar_simulation/urdf/robocar.urdf.xacro" ]; then
    echo "✓ URDF file found"
else
    echo "✗ URDF file missing"
fi

if [ -f "$WORKSPACE_DIR/src/robocar_simulation/launch/simulation.launch.py" ]; then
    echo "✓ Launch file found"
else
    echo "✗ Launch file missing"
fi

if [ -f "$WORKSPACE_DIR/src/robocar_simulation/worlds/maze.sdf" ]; then
    echo "✓ World file found"
else
    echo "✗ World file missing"
fi
echo ""

# Check running ROS nodes if any
echo "=== Checking running ROS nodes ==="
RUNNING_NODES=$(ros2 node list 2>/dev/null)
if [ -z "$RUNNING_NODES" ]; then
    echo "No ROS nodes currently running"
else
    echo "Running nodes:"
    echo "$RUNNING_NODES"
fi
echo ""

# Check available topics
echo "=== Checking ROS topics ==="
TOPICS=$(ros2 topic list 2>/dev/null)
if [ -z "$TOPICS" ]; then
    echo "No ROS topics available"
else
    echo "Available topics:"
    echo "$TOPICS"
    
    # Check specifically for command topics
    if echo "$TOPICS" | grep -q "cmd_vel"; then
        echo "✓ cmd_vel topic found"
    else
        echo "✗ cmd_vel topic missing"
    fi
    
    if echo "$TOPICS" | grep -q "/model/robocar/cmd_vel"; then
        echo "✓ /model/robocar/cmd_vel topic found"
    else
        echo "✗ /model/robocar/cmd_vel topic missing"
    fi
fi
echo ""

# Check Gazebo topics if Gazebo is running
echo "=== Checking Gazebo topics ==="
if pgrep -x "gz" > /dev/null; then
    GZ_TOPICS=$(gz topic -l 2>/dev/null)
    if [ -z "$GZ_TOPICS" ]; then
        echo "No Gazebo topics available"
    else
        echo "Gazebo is running with these topics:"
        echo "$GZ_TOPICS" | grep -i "cmd_vel\|robocar"
    fi
else
    echo "Gazebo is not running"
fi
echo ""

echo "=== Diagnostic Summary ==="
echo "If any issues were found above, fix them and try again."
echo "To test your RoboCar, run these commands:"
echo ""
echo "1. Launch the simulation:"
echo "   ros2 launch robocar_simulation simulation.launch.py"
echo ""
echo "2. Run the test script:"
echo "   ros2 run robocar_simulation robocar_test"
echo ""
echo "3. If that doesn't work, try direct control:"
echo "   ros2 run robocar_simulation harmonium_velocity_test"
echo ""