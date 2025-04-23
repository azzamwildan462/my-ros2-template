#!/usr/bin/env python3
import subprocess
import os
import signal

# Root dir of the workspace
workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
bin_dir = os.path.join(workspace_dir, "bin")

# Define binaries to launch
# Each entry is: (name, command list)
nodes_to_launch = [
    ("HAL_gpio", [os.path.join(bin_dir, "HAL_gpio")]),
    ("master", [os.path.join(bin_dir, "master")]),
    ("hello_world", ["python3", os.path.join(bin_dir, "hello_world.py")]),
]

processes = []

def main():
    try:
        for name, cmd in nodes_to_launch:
            print(f"[LAUNCH] Starting {name} → {' '.join(cmd)}")
            proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
            processes.append((name, proc))

        # Wait for all to finish
        for name, proc in processes:
            proc.wait()

    except KeyboardInterrupt:
        print("\n[LAUNCH] Caught Ctrl+C! Terminating all nodes...")
        for name, proc in processes:
            print(f"[LAUNCH] Stopping {name}")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception as e:
                print(f"Error killing {name}: {e}")

if __name__ == "__main__":
    main()
