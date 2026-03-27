# -*- python -*-



load("//tools/workspace:github_archive.bzl", "github_archive")

def sophus_repository(name):
    github_archive(
        name = name,
        repo = "strasdat/Sophus",
        commit = "ef9551ff429899b5adae66eabd5a23f165953199",
        sha256 = "b5a260f5db7ace1718e9bd44c21fb1a8588e1fb05ae0da29e04bb0eca1906143",
        build_file = Label("//tools/workspace/sophus:package.BUILD"),
    )
