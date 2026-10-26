#!/bin/bash
# Regenerate third_party_patches/ from the nested upstream clones in this workspace.
# For every baseline repo we modified, it stores:  upstream.txt (url + pinned commit), changes.diff (our edits to tracked
# files) and added/ (code files we created inside that tree). Checkpoints, outputs, results, symlinks and caches are skipped.
# Run from anywhere:  bash third_party_patches/export_patches.sh
set -u
WS=$(cd "$(dirname "$0")/.." && pwd); OUT=$WS/third_party_patches
SKIP='ckpt/|checkpoint|results/|outputs/|eval_[a-z0-9]+/|__pycache__|\.pyc$|\.pt$|\.pth$|\.ckpt$|\.safetensors$|\.mp4$|\.avi$|\.png$|\.jpg$|\.npy$|\.egg-info|CARLA_0915|Bench2DriveZoo|(^|/)test\.json$|\.log$'
export_repo() {   # export_repo <name> <path>
    local name=$1 repo=$2 dst=$OUT/$1
    [ -d "$repo/.git" ] || { echo "skip $name: not a git repo"; return; }
    rm -rf "$dst"; mkdir -p "$dst/added"
    ( cd "$repo"
      echo "url: $(git remote get-url origin)" > "$dst/upstream.txt"; echo "commit: $(git rev-parse HEAD)" >> "$dst/upstream.txt"
      git diff --no-color -- . ':(exclude)*.png' ':(exclude)*.jpg' > "$dst/changes.diff"
      n=0
      while IFS= read -r f; do
          [ -z "$f" ] && continue; [ -L "$f" ] && continue; [ -d "$f" ] && continue
          echo "$f" | grep -Eq "$SKIP" && continue
          mkdir -p "$dst/added/$(dirname "$f")"; cp -p "$f" "$dst/added/$f"; n=$((n+1))
      done < <(git ls-files --others --exclude-standard)
      echo "$name: $(git diff --stat | tail -1 | sed 's/^ *//'); $n added files" )
}
export_repo Bench2Drive "$WS/Bench2Drive"
export_repo LEAD        "$WS/LEAD"
export_repo CaRL        "$WS/CaRL"
export_repo SimLingo    "$WS/SimLingo"
export_repo DriveMoE    "$WS/DriveMoE"
export_repo BridgeDrive "$WS/BridgeDrive"
export_repo WA-JEPA     "/home/all/WA-JEPA"
find "$OUT" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null; echo "written to $OUT"
