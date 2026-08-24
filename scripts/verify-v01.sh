#!/usr/bin/env bash
set -euo pipefail
repository_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repository_dir"
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo test --all-targets
node --check web/demo.js
./scripts/build-wasm.sh
node scripts/test-wasm-controller-snapshot.mjs
node scripts/benchmark-wasm.mjs
echo "v0.1 acceptance gate passed"
