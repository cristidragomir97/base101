#!/usr/bin/env bash
#
# Bring base101's MoveIt semantics back in step after the mod101 arm changed.
#
# WHY THIS EXISTS. base101 embeds the mod101 arm through the `mod101_arm` macro,
# so a rail length, a servo mount or a tool chosen in mod101's configurator
# changes the geometry of *this* robot too — and with it which link pairs can
# never touch. base101's chassis matrices are therefore stale the moment the arm
# is reconfigured.
#
# The mod101 configurator used to reach across and regenerate these for you.
# That inverted the dependency: the arm had to know where its consumers lived,
# which breaks the moment there are two of them, or one is on another machine,
# or it isn't base101 at all. So the arm now regenerates only itself and tells
# you to run this. This script is that step, and it lives here because base101
# is the thing that knows how base101 is built.
#
# USAGE
#   ./scripts/sync_arm_change.sh                    # all tools
#   ./scripts/sync_arm_change.sh --tool jaws        # one tool
#   ./scripts/sync_arm_change.sh --trials 4000000   # more thorough
#
# Any arguments are passed straight through to gen_collision_matrix.py.
#
#   MOD101_WS=~/src/mod101 ./scripts/sync_arm_change.sh   # non-default underlay
#
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE101_WS="$(cd "$PKG_DIR/../../.." && pwd)"
MOD101_WS="${MOD101_WS:-$HOME/robots/mod101}"

die() { echo "sync_arm_change: $*" >&2; exit 1; }

[ -n "${ROS_DISTRO:-}" ] || die "source /opt/ros/<distro>/setup.bash first"

# The mod101 underlay must come first: this robot's URDF and SRDF both include
# packages from it, and its config xacro is the single source of truth for the
# four build args.
MOD101_SETUP="$MOD101_WS/install/setup.bash"
[ -f "$MOD101_SETUP" ] || die "mod101 underlay not built at $MOD101_WS
  build it, or point MOD101_WS at the right workspace:
    MOD101_WS=/path/to/mod101 $0 $*"

BASE101_SETUP="$BASE101_WS/install/setup.bash"
[ -f "$BASE101_SETUP" ] || die "base101 not built at $BASE101_WS — run colcon build first"

# colcon's generated setup.bash reads COLCON_TRACE unguarded, so `set -u` kills
# it before it does anything. Relax nounset across the sourcing only.
set +u
# shellcheck source=/dev/null
source "$MOD101_SETUP"
# shellcheck source=/dev/null
source "$BASE101_SETUP"
set -u

echo "sync_arm_change: mod101 underlay $MOD101_WS"
echo "sync_arm_change: regenerating base101 chassis collision matrices"
python3 "$PKG_DIR/scripts/gen_collision_matrix.py" "$@"

cat <<'EOF'

Done. The build args were read from the mod101 underlay, so these matrices now
match whatever the configurator last saved.

These matrices live in an ament_cmake package, so --symlink-install has them
linked back to src/ and no rebuild is needed. (URDF/xacro edits are different:
those packages are ament_python and install by copy, so they DO need a build.)
Relaunch to pick the new matrices up:

    ros2 launch base101_arm_moveit_config demo.launch.py
EOF
