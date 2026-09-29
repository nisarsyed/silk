# Browser motion lab

Build with the pinned WASM toolchain and assemble the renderer-study output as
described in [bench/render/README.md](../../bench/render/README.md). Then serve
`build/render-study` over HTTP and open `sandbox.html`:

```sh
python3 -m http.server 8000 --bind 127.0.0.1 --directory build/render-study
```

The initial sandbox uses Canvas 2D on the main thread. Pyramid, Rain, and
Chains start with the frozen default bodies. The interactive profile allocates
4096 body, 32768 contact, and 96 joint slots before building any scene; it is
separate from benchmark runs. Choose Circle or Box, then tap the canvas to add
one 1 kg body. Drag an existing body to apply the engine's fixed-step tether.
The interface visibly stops spawning at the fixed body limit. Reset and scene
changes retain the interactive capacities. Pause, single-step, sleep, and
diagnostic overlays are available without a keyboard.

The browser can drop simulation time when physics exceeds device capacity,
which the page reports. It does not silently lower scene counts or engine
limits. The deferred renderer/worker comparison and iPhone Rain quality failure
remain open; this build is a preview for interaction and correctness, not a
performance pass or deployment artifact.
