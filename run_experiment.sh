#!/usr/bin/env bash
# Usage: ./run_experiment.sh res://experiments/ablation.tres 50 experiments/ablation.csv [--log-transitions] [--time-scale 8]
# --fixed-fps 60 (an engine flag, so it goes before the scene) drops real-time sync: the run is
# CPU-bound. BatchRunner keeps the physics step at 1/60 s at any --time-scale, so rows do not
# depend on the scale or on --fixed-fps.
set -eu
cd "$(dirname "$0")"
CONFIG=$1; ROUNDS=$2; OUT=$3; shift 3
./Godot_v4.7.2-stable_linux.x86_64 --headless --path . --fixed-fps 60 res://scenes/batch.tscn -- \
  --config "$CONFIG" --rounds "$ROUNDS" --out "$OUT" "$@"
