#!/bin/bash

# Function to check if a command exists
command_exists () {
    type "$1" &> /dev/null ;
}

# Update package list and install necessary packages
sudo apt-get update

# Install catkin tools if they are not installed
if ! command_exists catkin_make; then
    echo "catkin_make not found, installing catkin tools..."
    sudo apt-get install -y ros-$(rosversion -d)-catkin
else
    echo "catkin_make is already installed."
fi

# Create the directory if it does not exist
mkdir -p ~/turtlebot_ws

# Move the src directory into the turtlebot_ws directory
# Check if the src directory exists before attempting to move it
if [ -d "setup/src" ]; then
    mv setup/src ~/turtlebot_ws/
else
    echo "Source directory setup/src does not exist."
    exit 1
fi

# Change to the turtlebot_ws directory
cd ~/turtlebot_ws

# Run catkin_make
catkin_make

# Check if catkin_make was successful
if [ $? -eq 0 ]; then
    echo "Workspace built successfully."
else
    echo "Failed to build the workspace."
    exit 1
fi

#source to workspace
source ~/turtlebot_ws/devel/setup.bash
