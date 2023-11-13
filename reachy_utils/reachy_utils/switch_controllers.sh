#!/bin/bash

# Define the list of position controllers
position_controllers=(
    "gripper_forward_position_controller"
    "l_arm_forward_position_controller"
    "neck_forward_position_controller"
    "r_arm_forward_position_controller"
)

# Define the list of trajectory controllers
trajectory_controllers=(
    "head_controller"
    "left_arm_controller"
    "left_gripper_controller"
    "right_arm_controller"
    "right_gripper_controller"
)

# Function to stop controllers
stop_controllers() {
    for controller in "${@}"; do
        echo "Stopping ${controller}..."
        ros2 control set_controller_state "$controller" inactive
    done
}

# Function to start controllers
start_controllers() {
    for controller in "${@}"; do
        echo "Starting ${controller}..."
        ros2 control set_controller_state "$controller" active
    done
}

# Display usage information
usage() {
    echo "Usage: $0 [position|trajectory]"
    echo "  position: Stops trajectory controllers and starts position controllers."
    echo "  trajectory: Stops position controllers and starts trajectory controllers."
}

# Check for correct number of arguments
if [ "$#" -ne 1 ]; then
    echo "Error: Incorrect number of arguments."
    usage
    exit 1
fi

# Main logic based on the argument
case "$1" in
    position)
        stop_controllers "${trajectory_controllers[@]}"
        start_controllers "${position_controllers[@]}"
        ;;
    trajectory)
        stop_controllers "${position_controllers[@]}"
        start_controllers "${trajectory_controllers[@]}"
        ;;
    *)
        echo "Error: Invalid argument."
        usage
        exit 1
        ;;
esac

echo "Controller management complete. Here is current controllers status:"
ros2 control list_controllers
