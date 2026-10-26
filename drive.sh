#!/bin/bash
# Manual driving in a pygame window (needs a running server: bash carla_server.sh).
# MIRRORS: left/right rear-view mirrors are drawn in the bottom corners by default (MIRRORS=0 hides).
# WHEEL: optional on-screen steering wheel — MOUSE_STEER=1 bash drive.sh ... (click + drag around its centre; RMB = brake)
# Keys: W/S throttle/brake, A/D steer, Q reverse, Space handbrake, P autopilot, TAB change camera,
#       ` next sensor, C weather, Backspace new vehicle, N map layers, R record, H help, ESC quit.
# Usage: bash drive.sh [--map Town12] [--filter vehicle.tesla.model3] [--res 1920x1080]
#   --map is handled here (loads the map on the server; ~30 s small towns, ~90 s Town12/13),
#   everything else is passed to manual_control.py.
PY=/home/all/miniconda3/envs/b2d/bin/python
MAP=""; ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --map) MAP="$2"; shift 2 ;;
    --map=*) MAP="${1#--map=}"; shift ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
if [ -n "$MAP" ]; then
  echo ">>> loading $MAP on the server (this can take a while for Town12/13)..."
  $PY - "$MAP" <<'PY'
import sys, carla
c = carla.Client('localhost', 2000); c.set_timeout(300.0)
want = sys.argv[1]
cur = c.get_world().get_map().name.split('/')[-1]
if cur != want:
    c.load_world(want)
print('>>> map:', c.get_world().get_map().name.split('/')[-1])
PY
fi
cd /home/all/carla_ws/carla/PythonAPI/examples
exec $PY manual_control.py --host localhost --port 2000 "${ARGS[@]}"
