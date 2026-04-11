#!/usr/bin/env python3
"""
GNSS Positioning Engine — Main Entry Point.

A software-defined GNSS positioning engine that computes position
from raw RTCM3 measurements (pseudorange, carrier phase, ephemeris)
using any GNSS receiver that outputs RTCM3 as measurement front-end.

Supports: GPS (L1/L5), Galileo (E1/E5a), GLONASS (L1/L2), BeiDou (B1/B2a)
Modes:    SPS (implemented), DGNSS (stub), RTK (stub)

Usage:
    python main.py              # Launch PyQt5 GUI
    python main.py --headless   # Run without GUI (future)
"""

import sys
import os
import argparse
import logging


def _fix_qt_plugin_path():
    """Ensure Qt can find its platform plugins on macOS.

    PyQt5/6 installed via pip sometimes can't locate the cocoa plugin.
    This sets QT_QPA_PLATFORM_PLUGIN_PATH before QApplication is created.
    """
    if os.environ.get('QT_QPA_PLATFORM_PLUGIN_PATH'):
        return  # Already set by user

    # Try PyQt6 first, then PyQt5
    for pkg_name, subdirs in [
        ('PyQt6', ['Qt6/plugins/platforms', 'Qt/plugins/platforms']),
        ('PyQt5', ['Qt5/plugins/platforms', 'Qt/plugins/platforms']),
    ]:
        try:
            pkg = __import__(pkg_name)
            for subdir in subdirs:
                qt_dir = os.path.join(pkg.__path__[0], *subdir.split('/'))
                if os.path.isdir(qt_dir):
                    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = qt_dir
                    return
        except ImportError:
            continue


def main():
    parser = argparse.ArgumentParser(
        description="GNSS Positioning Engine"
    )
    parser.add_argument(
        '--headless', action='store_true',
        help='Run without GUI (not yet implemented)'
    )
    parser.add_argument(
        '--log-level', default='INFO',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
        help='Logging level'
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
        datefmt='%H:%M:%S',
    )

    if args.headless:
        print("Headless mode not yet implemented.")
        print("Use the GUI for now: python main.py")
        sys.exit(1)

    _fix_qt_plugin_path()

    from gnss_positioning.ui.main_window import run_app
    run_app()


if __name__ == '__main__':
    main()