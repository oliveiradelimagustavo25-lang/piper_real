#!/usr/bin/env bash
# Stop local ROS processes belonging to one ROS_DOMAIN_ID.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$script_dir/build/clear.py" "$@"
