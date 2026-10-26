#!/bin/bash
# Start the CARLA server (port 2000). Usage: bash carla_server.sh [extra CarlaUE4 args]
# Default is offscreen (no server window; clients render their own views).
# Pass -windowed to see the server's spectator window instead of -RenderOffScreen.
# QUALITY=Low bash carla_server.sh   -> faster rendering (helps the 11-sensor BridgeDrive agent); default Epic.
cd /home/all/carla_ws/carla
if [[ " $* " == *" -windowed "* ]]; then MODE="-windowed -ResX=1280 -ResY=720"; ARGS="${*//-windowed/}"; else MODE="-RenderOffScreen"; ARGS="$*"; fi
exec ./CarlaUE4.sh $MODE -nosound -carla-rpc-port=2000 -quality-level=${QUALITY:-Epic} $ARGS
