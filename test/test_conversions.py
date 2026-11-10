"""Run the actual SDK-to-ROS C++ conversion checks through a Python entry point."""
import os
from pathlib import Path
import subprocess


def test_native_conversions():
    default = Path(__file__).resolve().parents[1] / 'build/vidu_ros2_wrapper/conversion_probe'
    executable = Path(os.environ.get('VIDU_CONVERSION_PROBE', str(default)))
    assert executable.is_file(), 'Build vidu_ros2_wrapper with BUILD_TESTING=ON first'
    result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
