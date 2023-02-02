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
    commit = "1748b4bf4f7f39fa4cfac4dc45b4d4d1bb700944",
    remote = "https://github.com/ghorn/bb3d.git",
    shallow_since = "1638764865 -0800",
)
