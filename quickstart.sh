#!/bin/bash
# Get the directory where the script is located
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Launch Terminator with the specific layout and working directory
terminator --working-directory="$REPO_DIR" --layout=rins