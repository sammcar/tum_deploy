# -*- python -*-



load("//tools/workspace:github_archive.bzl", "github_archive")

def moteus_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/moteus",
        commit = "fe3a19f2c05de9aa36017f7e21f95a2bca362c75",
        sha256 = "361969521fc0165f78d78fb918642562b58f5b55a42fc5428aece70906e8daeb",
    )
