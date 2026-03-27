# -*- python -*-


load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")


def rules_pkg_repository(name):
    http_archive(
        name = name,
        urls = [
            "https://github.com/bazelbuild/rules_pkg/releases/download/1.0.1/rules_pkg-1.0.1.tar.gz",
        ],
        sha256 = "d20c951960ed77cb7b341c2a59488534e494d5ad1d30c4818c736d57772a9fef",
    )
