There are some system dependencies needed. Here is an incomplete list:

> sudo apt install libglfw3-dev libboost-stacktrace-dev libglm-dev libglew-dev libfreetype-dev libeigen3-dev libsoil-dev

You need bazel to build the project. You can install it by downloading bazelisk from https://github.com/bazelbuild/bazelisk/releases
and moving it to /usr/local/bin/bazel. Don't forget to `chmod +x /path/to/bazelisk`.

To solve random path finding problems and see them in 3d, try running:

> bazel run -c opt //:r3_vis
> bazel run -c opt //:se2_vis
