# -*- python -*-


load("//tools/workspace:github_archive.bzl", "github_archive")

def pi3hat_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/pi3hat",
        commit = "958e4c39a2b5fed06d32ff1887ef9c38a479320a",
        sha256 = "a19bfa73c6b0334efc3d7e4f5c772dd6396ce69c5d236699c5f8f5ed20de17a5",
    )
