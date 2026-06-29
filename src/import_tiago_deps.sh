#!/bin/bash
# Script to clone TIAGo dependencies for ROS 2 Jazzy

cd "$(dirname "$0")"

mkdir -p tiago_harmonic_deps
cd tiago_harmonic_deps

echo "Cloning TIAGo Harmonic dependencies..."

git clone -b main https://github.com/Tiago-Harmonic/br2_gazebo_worlds.git
git clone -b master https://github.com/Tiago-Harmonic/launch_pal.git
git clone -b jazzy https://github.com/Tiago-Harmonic/pal_gripper.git
git clone -b jazzy https://github.com/Tiago-Harmonic/pal_hey5.git
git clone -b main https://github.com/Tiago-Harmonic/pal_maps.git
git clone -b jazzy https://github.com/Tiago-Harmonic/pal_robotiq_gripper.git
git clone -b jazzy https://github.com/Tiago-Harmonic/pal_urdf_utils.git
git clone -b jazzy https://github.com/Tiago-Harmonic/play_motion2.git
git clone -b jazzy https://github.com/Tiago-Harmonic/pmb2_navigation.git
git clone -b jazzy https://github.com/Tiago-Harmonic/pmb2_robot.git
git clone -b jazzy https://github.com/Tiago-Harmonic/tiago_moveit_config.git
git clone -b jazzy https://github.com/Tiago-Harmonic/tiago_navigation.git
git clone -b jazzy https://github.com/Tiago-Harmonic/omni_base_robot.git
git clone -b jazzy https://github.com/Tiago-Harmonic/tiago_robot.git
git clone -b jazzy https://github.com/Tiago-Harmonic/tiago_simulation.git

echo "All dependencies cloned successfully!"
