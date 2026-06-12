#!/usr/bin/env bash
# base101 dev stack in one tmux session: sim + slam + nav + robocore agent.
#
#   ./scripts/stack.sh            # start (or re-attach to) the stack
#   ./scripts/stack.sh kill       # tear the whole session down
#
# One window, 2x2 panes, all consoles visible:
#   ┌─────────────┬─────────────┐
#   │ gazebo      │ slam        │
#   ├─────────────┼─────────────┤
#   │ nav         │ agent       │
#   └─────────────┴─────────────┘
# Ctrl-b + arrows to move between panes; Ctrl-b z to zoom one.

set -euo pipefail

SESSION=base101
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="$WS/../engine/profiles/base101_full.yaml"
SETUP="source $WS/install/setup.bash"

if [[ "${1:-}" == "kill" ]]; then
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "killed $SESSION"
    exit 0
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "session '$SESSION' already running; attaching (use '$0 kill' to reset)"
    exec tmux attach -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n stack -x 220 -y 50

# Pane indices renumber on every split; capture the stable pane IDs.
GAZEBO=$(tmux display-message -p -t "$SESSION:stack" '#{pane_id}')
SLAM=$(tmux split-window -h -t "$GAZEBO" -P -F '#{pane_id}')
NAV=$(tmux split-window -v -t "$GAZEBO" -P -F '#{pane_id}')
AGENT=$(tmux split-window -v -t "$SLAM" -P -F '#{pane_id}')

tmux send-keys -t "$GAZEBO" \
    "$SETUP && ros2 launch base101_gazebo gazebo.launch.py tower:=true arms:=true rosboard:=false" Enter
tmux send-keys -t "$SLAM" \
    "$SETUP && sleep 10 && ros2 launch base101_slam slam.launch.py use_sim_time:=true" Enter
tmux send-keys -t "$NAV" \
    "$SETUP && sleep 10 && ros2 launch base101_nav nav.launch.py use_sim_time:=true" Enter
tmux send-keys -t "$AGENT" \
    "$SETUP && sleep 20 && ros2 run robocore_agent agent --profile $PROFILE --ros-args -p use_sim_time:=true" Enter

tmux select-layout -t "$SESSION:stack" tiled
tmux select-pane -t "$GAZEBO"

if [[ -n "${TMUX:-}" ]]; then
    echo "already inside tmux: 'tmux switch-client -t $SESSION'"
else
    exec tmux attach -t "$SESSION"
fi
