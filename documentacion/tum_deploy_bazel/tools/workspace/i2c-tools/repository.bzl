# -*- python -*-



load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")


def i2c_tools_repository(name):
    http_archive(
        name = name,
        urls = [
            "https://mirrors.edge.kernel.org/pub/software/utils/i2c-tools/i2c-tools-4.0.tar.xz",
        ],
        sha256 = "d900ca1c11c51ea20caa50b096f948008b8a7ad832311b23353e21baa7af28d6",
        strip_prefix = "i2c-tools-4.0",
        build_file = Label("//tools/workspace/i2c-tools:package.BUILD"),
    )
