#!/usr/bin/env bash
set -e
DATA=${1:-../data}
OUT=${2:-results}
PARAMS=${3:-}
ARGS=()
[ -n "$PARAMS" ] && ARGS=(--params "$PARAMS")
for bag in "$DATA"/for_hackathon/* "$DATA"/synthetic/*; do
  [ -f "$bag/metadata.yaml" ] || continue
  python tools/run_bag.py "$bag" --out "$OUT/$(basename "$bag").jsonl" "${ARGS[@]}" &
done
wait
python tools/evaluate.py "$OUT"/*.jsonl
