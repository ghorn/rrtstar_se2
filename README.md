There are some system dependencies needed. Here is an incomplete list:

> sudo apt install libglfw3-dev libboost-stacktrace-dev libglm-dev libglew-dev libfreetype-dev libeigen3-dev libsoil-dev

To solve random path finding problems and see them in 3d, try running:

> ./bazelisk-linux-amd64 run -c opt //:r3_vis
> ./bazelisk-linux-amd64 run -c opt //:se2_vis

bazelisk is a program that acts as a pass-through to bazel. It downloads bazel to ~/.cache/bazelisk
and calls it transparently from there. A bazelisk binary is committed to this repository.
