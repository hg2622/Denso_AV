#!/bin/bash
# Kill zombie evaluators: the batches run one leaderboard_evaluator at a time; an older second one is a leftover (e.g. waiting
# 600 s on a CARLA that segfaulted) and would hold GPU memory. Also kills an evaluator whose own CARLA server is gone for >90 s.
declare -A GONE
while true; do
  PIDS=$(pgrep -f "leaderboard_evaluato[r]" | sort -n); N=$(echo "$PIDS" | grep -c .)
  if [ "$N" -ge 2 ]; then OLD=$(echo "$PIDS" | head -1); echo "[$(date +%H:%M:%S)] killing zombie evaluator $OLD"; kill -9 $OLD; fi
  for P in $PIDS; do
    # the evaluators relaunch CARLA on a bumped port (find_free_port): do not match the --port arg, require no CARLA server at all
    if ! pgrep -f "CarlaUE4-Linux-Shippin[g]" >/dev/null; then GONE[$P]=$(( ${GONE[$P]:-0} + 1 )); else GONE[$P]=0; fi
    if [ "${GONE[$P]:-0}" -ge 9 ]; then echo "[$(date +%H:%M:%S)] evaluator $P: no CARLA server for 90 s, killing"; kill -9 $P; GONE[$P]=0; fi
  done
  # orphaned evaluator CARLA servers (ports >= 8840; the user's own server on 2000 is never touched): kill after 60 s without any evaluator
  if [ "$N" -eq 0 ]; then NOEVAL=$(( ${NOEVAL:-0} + 1 )); else NOEVAL=0; fi
  if [ "${NOEVAL:-0}" -ge 6 ]; then
    for L in $(pgrep -f "CarlaUE4-Linux-Shippin[g]"); do PT=$(tr '\0' ' ' < /proc/$L/cmdline 2>/dev/null | grep -oE "rpc-port=[0-9]+" | cut -d= -f2); [ -n "$PT" ] && [ "$PT" -ge 8840 ] && { echo "[$(date +%H:%M:%S)] killing orphan CARLA $L (port $PT, no evaluator)"; kill -9 $L; }; done
    NOEVAL=0
  fi
  sleep 10
done
