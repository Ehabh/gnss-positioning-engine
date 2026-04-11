#!/bin/bash
# GNSS Positioning Engine — macOS launcher
# Double-click this file in Finder to launch the app.
# The app will appear as "GNSS Engine" in the Dock.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Prefer the local venv if present
if [ -f ".venv/bin/python" ]; then
    PYTHON=".venv/bin/python"
elif command -v python3 &>/dev/null; then
    PYTHON="python3"
else
    osascript -e 'display alert "Python not found" message "Install Python 3.10+ and run: pip install PySide6 numpy pyserial" as critical'
    exit 1
fi

exec "$PYTHON" main.py "$@"
