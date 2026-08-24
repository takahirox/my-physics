#!/usr/bin/env bash
set -euo pipefail
repository_dir="$(cd "$(dirname "$0")/.." && pwd)"
rustc_bin="$(rustup which rustc)"
cargo_bin="$(rustup which cargo)"
RUSTC="$rustc_bin" "$cargo_bin" build --manifest-path "$repository_dir/Cargo.toml" --target wasm32-unknown-unknown --release --lib
cp "$repository_dir/target/wasm32-unknown-unknown/release/my_physics.wasm" "$repository_dir/web/physics.wasm"
echo "Built web/physics.wasm"
