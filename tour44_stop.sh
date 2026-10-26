#!/bin/bash
# stop a running tour44 (the current route is abandoned; re-running tour44.sh resumes from it)
for pid in $(pgrep -f "bash .*tour4[4]\.sh"); do [ "$pid" != "$$" ] && [ "$pid" != "$PPID" ] && kill $pid 2>/dev/null; done
pkill -9 -f "record_her[o].py"; pkill -9 -f "leaderboard_evaluato[r]"; pkill -9 -f "CarlaUE4-Linux-Shipp[i]ng"; sleep 2; echo "tour44 stopped"
