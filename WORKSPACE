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
    commit = "14c9303b3d9e86ac66e2699e98e3d5ecc6386239",
    shallow_since = "1638471161 -0800",
)

http_archive(
    name = "nanoflann",
    build_file = "@rrtstar_se2//third_party:BUILD.nanoflann",
    sha256 = "e100b5fc8d72e9426a80312d852a62c05ddefd23f17cbb22ccd8b458b11d0bea",
    strip_prefix = "nanoflann-1.3.2",
    urls = ["https://github.com/jlblancoc/nanoflann/archive/refs/tags/v1.3.2.tar.gz"],
)
