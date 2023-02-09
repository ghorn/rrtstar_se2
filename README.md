# Path Finding with RRT\* in R^3 and SE^2

[Live demo](https://ghorn.github.io/rrtstar_se2/)

The primary way to run this is to compile the C++ to WebAssembly with emscripten, and then load index.html.
A github action deploys a github page which is hosted at https://ghorn.github.io/rrtstar_se2/ (live demo above).

## Running locally for development

```
# install emscripten
sudo apt install emscripten

# install bazel to ./bazel
wget https://github.com/bazelbuild/bazelisk/releases/download/v1.16.0/bazelisk-linux-amd64 -O bazel
chmod +x bazel

# build the webassembly
./bazel build -c opt //vis:shim_wasm

# launch a webserver
python3 -m http.server
```

Then go to http://localhost:8000/index.html in your browser.
