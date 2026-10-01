# Boids

Unity's ECS Boids sample ported to the Kaigen Engine, a high performance, cross-platform 3D engine written in C. It
compares the two ECS implementations running the same scene in the browser.

[Play it on kaigen3d.com](https://kaigen3d.com/demos/boids/) · [Register for the closed beta.](https://kaigen3d.com/)

![Boids](preview.png)

The Unity project it was ported from is the `EntitiesSamples/Assets/Boids` sample in our fork of Unity's samples:
[Kaigen-Technologies/EntityComponentSystemSamples](https://github.com/Kaigen-Technologies/EntityComponentSystemSamples).

## Running it

The project needs the Kaigen Engine, which is in closed beta. `hz` is a link to your engine checkout at the project
root (it is not part of this repository).

```
hz/hzbuild run                   # native debug build, then run
hz/hzbuild --release run         # native optimized build
hz/hzbuild wasm --release run    # WebAssembly build, served at http://localhost:3000
```

Hold the right mouse button to look around and fly with W A S D. The buttons at the bottom add or remove 5,000 fish.
