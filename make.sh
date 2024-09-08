#!/bin/bash

set -e # Exit immediately if a command exits with a non-zero status.
cd "$(dirname "$0")" # Change directory to the script location.

find src -type f -wholename */scripts/*.py -exec chmod +x {} \; # Make all python scripts executable.

colcon build --symlink-install --executor parallel --parallel $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release