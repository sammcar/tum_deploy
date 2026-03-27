# -*- python -*-


load("//tools/workspace:github_archive.bzl", "github_archive")

def bazel_deps_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/bazel_deps",
        commit = "9e640212a87ccc5fa5837df35c063ba2261d4e76",
        sha256 = "79030e39f06961deea88e292fd2399df8a67d6376295bb3d1fc9a93f97563e07",
    )
