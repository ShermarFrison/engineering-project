#!/usr/bin/env bash
# Usage: ./run_experiment.sh res://experiments/ablation.tres 50 experiments/ablation.csv [--log-transitions] [--time-scale 8]
set -eu
cd "$(dirname "$0")"
CONFIG=$1; ROUNDS=$2; OUT=$3; shift 3
./Godot_v4.7.2-stable_linux.x86_64 --headless --path . res://scenes/batch.tscn -- \
  --config "$CONFIG" --rounds "$ROUNDS" --out "$OUT" "$@"
