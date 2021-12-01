workspace(name = "rrtstar_se2")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

# googletest
http_archive(
    name = "com_google_googletest",
    sha256 = "b4870bf121ff7795ba20d20bcdd8627b8e088f2d1dab299a031c1034eddc93d5",
    strip_prefix = "googletest-release-1.11.0",
    urls = ["https://github.com/google/googletest/archive/release-1.11.0.tar.gz"],
)

#local_repository(
#    name = "bb3d",
#    path = "/home/greg/bb3d",
#)
git_repository(
    name = "bb3d",
    remote = "https://github.com/ghorn/bb3d.git",
    commit = "799334d625dcd3a5b362a3f62c618fcd3ccb80d8",
    shallow_since = "1638316800 -0800",
)
