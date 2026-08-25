#!/usr/bin/env bash
set -euo pipefail

repository_dir="$(cd "$(dirname "$0")/.." && pwd)"
artifact_dir="${1:-$repository_dir/target/maneuver-validation}"

cargo run --manifest-path "$repository_dir/Cargo.toml" --release --bin maneuver-validation -- \
  --summary --artifacts "$artifact_dir"
echo "Wrote maneuver validation artifacts to $artifact_dir"
