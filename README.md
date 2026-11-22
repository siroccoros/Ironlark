# Ironlark

Webots / ArduPilot drone simulation with a pine-forest world and ROS 2 sensor
recording. The flight check takes off, hovers and lands using simulated GPS.

Linux x86-64: Docker, [uv](https://docs.astral.sh/uv/), Git, `xvfb-run`, OpenGL,
a C/C++ build toolchain, and [Webots R2025a](https://github.com/cyberbotics/webots/releases/tag/R2025a)
extracted into `./webots/`. `doctor` lists missing renderer libraries.

```sh
./ironlark setup
./ironlark doctor
./ironlark build-renderer
./ironlark check
./ironlark build-world
./ironlark run
```

First setup needs internet and several GB of disk. Forest flights require the
patched renderer; rebuild after changing its patches or native sources.
`WEBOTS_HOME` overrides renderer selection.

```sh
./ironlark run --record
./ironlark replay runs/TIMESTAMP
./ironlark check-recording runs/TIMESTAMP
./ironlark --help
```

[Flight parameters](config/flight.parm) · [Sensors](config/sensors.json).
Run results and logs go in `runs/`.

For the offline drone-research atlas, run `python3 research/build_html.py`
and open `research/drone-autonomy-atlas.html`.

ArduPilot, Webots, Blender, Poly Haven and research material retain their
own terms. Setup saves ArduPilot's licence in its cache;
[meshoptimizer's MIT licence](native/meshoptimizer/LICENSE.md) is bundled.
