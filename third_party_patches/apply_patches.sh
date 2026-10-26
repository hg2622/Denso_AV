#!/bin/bash
# Recreate the modified baseline repos next to this workspace:  bash third_party_patches/apply_patches.sh [NAME ...]
# For each NAME (default: all) it clones the upstream at the pinned commit into ../NAME (WA-JEPA into $HOME/WA-JEPA, where the
# launchers expect it), applies changes.diff and copies the added/ files in. Model checkpoints are NOT included: fetch them
# from each project's own release page (see SETUP.txt).
set -eu
HERE=$(cd "$(dirname "$0")" && pwd); WS=$(cd "$HERE/.." && pwd)
names=("$@"); [ ${#names[@]} -eq 0 ] && names=(Bench2Drive LEAD CaRL SimLingo DriveMoE BridgeDrive WA-JEPA)
for name in "${names[@]}"; do
    src=$HERE/$name; [ -f "$src/upstream.txt" ] || { echo "no patch set for $name"; continue; }
    url=$(sed -n 's/^url: //p' "$src/upstream.txt"); commit=$(sed -n 's/^commit: //p' "$src/upstream.txt")
    dst=$WS/$name; [ "$name" = WA-JEPA ] && dst=$HOME/WA-JEPA
    if [ ! -d "$dst/.git" ]; then echo "cloning $url -> $dst"; git clone --recursive "$url" "$dst"; fi
    ( cd "$dst"; git checkout -q "$commit"
      if [ -s "$src/changes.diff" ]; then git apply --3way "$src/changes.diff" && echo "$name: diff applied" || echo "$name: diff needs manual merge (see git status)"; fi
      if [ -d "$src/added" ]; then (cd "$src/added" && find . -type f | while read -r f; do mkdir -p "$dst/$(dirname "$f")"; cp -p "$f" "$dst/$f"; done); echo "$name: added files copied"; fi )
done
echo "Bench2Drive/leaderboard/team_code expects symlinks to Bench2DriveZoo (tcp_b2d_agent.py, admlp_b2d_agent.py, planner.py): clone"
echo "https://github.com/Thinklab-SJTU/Bench2DriveZoo (branch tcp/admlp) into $WS/Bench2DriveZoo and link them (see SETUP.txt)."
