import os
import sys


sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
in_docker = any("docker" in line for line in open("/proc/self/mountinfo"))
if in_docker:
    build_dir = '/src/coloradar_tools/build'
else:
    cwd = os.getcwd()
    if cwd.endswith(os.path.join("coloradar_plus_processing_tools", "coloradar_tools")):
        build_dir = os.path.join(cwd, "..", "build")
    else:
        build_dir = os.path.join(cwd, "build")
sys.path.append(build_dir)

import coloradar_dataset_tools
import coloradar_cuda_tools
