#!/usr/bin/env bash
set -euo pipefail

# Synchronise the two source packages required by the active perception
# pipeline. Run without --apply first. The script never deletes remote files.

apply=false
host="jetson@jetson-desktop.local"
for argument in "$@"; do
    case "$argument" in
        --apply) apply=true ;;
        --host=*) host="${argument#*=}" ;;
        *) echo "Unknown argument: $argument" >&2; exit 2 ;;
    esac
done

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
workspace_source="$repo_root/ros2_ws/src"
workspace_destination="~/Desktop/ros2_ws/src"
packages=(imdl_msgs perception)
rsync_args=(--archive --itemize-changes --human-readable)
rsync_args+=(--exclude __pycache__/ --exclude .pytest_cache/ --exclude '*.pyc')

if ! "$apply"; then
    rsync_args+=(--dry-run)
    echo "Dry run only. Repeat with --apply to transfer files."
fi

for package in "${packages[@]}"; do
    rsync "${rsync_args[@]}" \
        "$workspace_source/$package/" \
        "$host:$workspace_destination/$package/"
done

if "$apply"; then
    ssh "$host" 'bash -lc "
        set -e
        source /opt/ros/humble/setup.bash
        cd ~/Desktop/ros2_ws/src/perception/models
        sha256sum -c yolo11n_seg_imdl.sha256
        cd ~/Desktop/ros2_ws
        colcon build --packages-select imdl_msgs perception --symlink-install
    "'
fi
