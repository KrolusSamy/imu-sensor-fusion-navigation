Import("env")
import subprocess
import sys
import os

def run_python_visualizer(source, target, env):
    print("\n--- Firmware Upload Successful! Launching Python Visualizer ---")
    
    # Path to your visualizer script relative to the project root
    visualizer_path = os.path.abspath(os.path.join(env.get("PROJECT_DIR"), "tools", "visualizer.py"))
    
    if os.path.exists(visualizer_path):
        # Spawns the visualizer using the same Python environment running PlatformIO
        # subprocess.Popen([sys.executable, visualizer_path])
        print(visualizer_path)
    else:
        print(f"ERROR: Could not find visualizer at {visualizer_path}")

# Hook this function to run ONLY after a successful upload
env.AddPostAction("upload", run_python_visualizer)