#!/bin/bash
# Small second window with two camera views of your car.   bash views.sh [view1] [view2]
# views: left right rear front bev chase   (default: left right = side mirrors)
exec /home/all/miniconda3/envs/b2d/bin/python -u /home/all/carla_ws/side_views.py "$@"
