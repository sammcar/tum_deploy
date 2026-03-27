# -*- python -*-



load("//tools/workspace:github_archive.bzl", "github_archive")

def mjlib_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/mjlib",
        commit = "ec386ea442ad6dc4d597f4ca80d2317a6d0ec283",
        sha256 = "b3ef858b7a0b7971fa1a2229a0481bfc8ddf0708a980c83f3885613e9e6ba7f6",
    )
