#!/usr/bin/env bash
# Runs the gdUnit4 suite headless. Exit 0 = pass, 100 = failures, 101 = orphan nodes (also a failure).
set -u
cd "$(dirname "$0")"
GODOT=./Godot_v4.7.2-stable_linux.x86_64
# Refresh the global script class cache so new class_name scripts resolve.
"$GODOT" --headless --path . --import >/dev/null 2>&1
"$GODOT" --headless --path . -s res://addons/gdUnit4/bin/GdUnitCmdTool.gd \
  --add res://tests --ignoreHeadlessMode "$@" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' | grep -vE '^\s*$'
exit "${PIPESTATUS[0]}"
