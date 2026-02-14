#!/bin/bash
# Get the directory where this script lives
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 1. Add our local 'libs' folder to the library path
export LD_LIBRARY_PATH="$DIR/libs":$LD_LIBRARY_PATH

# 2. Run the executable
"$DIR/FluxGeo"