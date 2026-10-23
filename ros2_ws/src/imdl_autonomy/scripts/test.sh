#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$workspace"

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

colcon test \
  --packages-select perception imdl_autonomy \
  --python-testing pytest \
  --return-code-on-test-failure \
  --event-handlers console_direct+
colcon test-result --verbose
