# -*- python -*-


load("//tools/workspace:github_archive.bzl", "github_archive")

def implot_repository(name):
    github_archive(
        name = name,
        repo = "epezent/implot",
        commit = "9894df49348a8de412fd3b52ff7bfb322dbeede2",
        sha256 = "f5f655ef6d6c167c978373dcd8a53f7188d99ce768a20156fc1e042167a7c296",
        build_file = Label("//tools/workspace/implot:package.BUILD"),
        patches = [Label("//tools/workspace/implot:empty-legend.diff")],
        patch_args = ["-p1"],
    )
