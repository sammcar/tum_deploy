# -*- python -*-



load("//tools/workspace:github_archive.bzl", "github_archive")

def rpi_bazel_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/rpi_bazel",
        commit = "3f9e6245972dfb48dffc04d5c520b799d07ea59c",
        sha256 = "fbbd18a9358b67eef091ebf8fffcfc03f0c3d5b582dd5b084081ce573c2dfda0",
    )
