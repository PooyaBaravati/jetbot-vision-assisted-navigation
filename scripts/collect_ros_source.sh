#!/usr/bin/env bash
# Collect a source-only copy of a ROS package on the machine where it exists.
# Usage: bash scripts/collect_ros_source.sh cl_camera_plugins jetbot
#        bash scripts/collect_ros_source.sh jetbot_mission vm
# Run from the relevant JetBot/VM Linux shell. Does not change running ROS nodes.
set -euo pipefail

PACKAGE="${1:-}"
LABEL="${2:-}"
case "$PACKAGE" in
  cl_camera_plugins|jetbot_mission) ;;
  *) echo "Usage: $0 {cl_camera_plugins|jetbot_mission} {jetbot|vm}" >&2; exit 2 ;;
esac
case "$LABEL" in
  jetbot|vm) ;;
  *) echo "Second argument must be jetbot or vm" >&2; exit 2 ;;
esac

SRC="$HOME/catkin_ws/src/$PACKAGE"
if [[ ! -d "$SRC" ]]; then
  echo "Package not found at $SRC" >&2
  echo "Searching for package directories in $HOME ..." >&2
  find "$HOME" -maxdepth 6 -type d -name "$PACKAGE" -print 2>/dev/null || true
  exit 1
fi

OUT="$HOME/${PACKAGE}_source_${LABEL}.tar.gz"
tar -czf "$OUT" \
  --exclude='__pycache__' --exclude='*.pyc' --exclude='.git' \
  --exclude='.ipynb_checkpoints' \
  --exclude='*.onnx' --exclude='*.pth' --exclude='*.pt' \
  --exclude='*.caffemodel' --exclude='*.engine' --exclude='*.uff' \
  --exclude='*.bag' --exclude='*.db3' --exclude='*.csv' \
  --exclude='*.pgm' --exclude='*.png' --exclude='*.jpg' --exclude='*.jpeg' \
  --exclude='*.mp4' --exclude='*.avi' \
  --exclude='*.pem' --exclude='*.key' --exclude='id_rsa*' \
  --exclude='*.env' --exclude='.env' \
  -C "$HOME/catkin_ws/src" "$PACKAGE"
echo "Source archive created: $OUT"
echo "Archive content (review before sharing):"
tar -tzf "$OUT"
