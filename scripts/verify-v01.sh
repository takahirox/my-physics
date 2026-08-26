#!/usr/bin/env bash
set -euo pipefail
repository_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repository_dir"
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo test --all-targets
cargo run --release --bin maneuver-validation -- --summary
node --check web/demo.js
node --check web/visual-config.mjs
node --check web/input-config.mjs
node --check web/race-state.mjs
node --check web/presentation-config.mjs
node --check web/audio-engine.mjs
node --test tests/web_visual_config.mjs tests/web_input_config.mjs tests/web_race_state.mjs tests/web_presentation_config.mjs tests/web_audio_engine.mjs
./scripts/build-wasm.sh
node scripts/test-wasm-controller-snapshot.mjs
node scripts/test-wasm-race.mjs
node scripts/test-wasm-simulation-lab.mjs
node scripts/benchmark-wasm.mjs
echo "v0.1 acceptance gate passed"
