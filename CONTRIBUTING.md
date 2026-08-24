# Contributing

Changes should preserve SI/radian/axis conventions, deterministic iteration order, headless operation and renderer independence. New physical approximations must be documented with their validity range and at least one measurable validation test.

Before opening a pull request, run `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings`, `cargo test --all-targets`, and the WASM release build. Do not add parallel or WebGPU work to the authoritative path without repeatability evidence and a documented fallback.

The project cannot accept outside contributions under an open-source grant until the license decision is completed.
