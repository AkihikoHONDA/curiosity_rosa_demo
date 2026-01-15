import os
import sys
from pathlib import Path


def pytest_configure():
    repo_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repo_root))
    rosidl_path = repo_root.parent / "build" / "curiosity_rosa_demo" / "rosidl_generator_py"
    if rosidl_path.exists():
        sys.path.insert(0, str(rosidl_path))
    log_dir = repo_root / ".ros" / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_LOG_DIR", str(log_dir))
