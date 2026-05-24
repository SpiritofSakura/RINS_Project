#!/usr/bin/env python3
"""Launch tile_detect + tile_classifier for manual testing.

Usage:
    ./classifier.py

Position the robot over a tile. The detector will find the tile,
wait 0.5 s, warp it to fronto-parallel, then the classifier runs
U-Net inference and shows the result in a window.
"""
import subprocess
import sys
import signal


def main():
    processes = []
    try:
        for node in ("tile_detect", "tile_classifier"):
            p = subprocess.Popen(
                ["ros2", "run", "task1", node],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
            processes.append(p)
            print(f"Started {node} (PID {p.pid})", file=sys.stderr)

        for p in processes:
            p.wait()
    except KeyboardInterrupt:
        print("\nShutting down...", file=sys.stderr)
        for p in processes:
            p.send_signal(signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()


if __name__ == "__main__":
    main()
