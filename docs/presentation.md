# Race presentation boundary

The race presentation consumes authoritative Rust/WASM telemetry but never
writes presentation values back into the physical plant. Camera response,
particles and audio therefore cannot change grip, steering, velocity, damage,
AI capability or timing.

`web/visual-config.mjs` contains bounded camera presets and telemetry response.
`web/presentation-config.mjs` maps tire scrub, water, hydroplaning, brake
temperature, impact, damage, speed and engine state to renderer-independent
effect intensities and capped emitter rates. Renderers must use object pools
whose maxima are declared in `EFFECT_LIMITS`; values are visual evidence, not
additional forces.

`web/audio-engine.mjs` maps the synthesis-neutral `AudioFrame` contract to a
small WebAudio graph. The audio context is created only by an explicit
`unlock()` call, which the UI invokes from a user gesture. It supports mute,
resource disposal and a silent fallback when WebAudio is unavailable. No
network audio assets are required.

The pure mappings sanitize missing, non-finite and out-of-range input. Node
tests cover rest behavior, dry smoke versus wet spray, monotonic response,
bounds, autoplay-safe initialization, mute and fallback behavior. Browser
integration should pass the plant's actual per-wheel fields rather than
inventing hidden visual state.
