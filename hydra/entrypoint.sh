#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /hydra_ws/install/setup.bash

exec "$@"
