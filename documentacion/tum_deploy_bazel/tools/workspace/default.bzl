# -*- python -*-


load("//tools/workspace/bazel_deps:repository.bzl", "bazel_deps_repository")
load("//tools/workspace/i2c-tools:repository.bzl", "i2c_tools_repository")
load("//tools/workspace/implot:repository.bzl", "implot_repository")
load("//tools/workspace/mjlib:repository.bzl", "mjlib_repository")
load("//tools/workspace/moteus:repository.bzl", "moteus_repository")
load("//tools/workspace/pi3hat:repository.bzl", "pi3hat_repository")
load("//tools/workspace/rpi_bazel:repository.bzl", "rpi_bazel_repository")
load("//tools/workspace/rules_pkg:repository.bzl", "rules_pkg_repository")
load("//tools/workspace/sophus:repository.bzl", "sophus_repository")

def add_default_repositories(excludes = []):
    if not native.existing_rule("com_github_mjbots_bazel_deps"):
        bazel_deps_repository(name = "com_github_mjbots_bazel_deps")
    if not native.existing_rule("i2c-tools"):
        i2c_tools_repository(name = "i2c-tools")
    if not native.existing_rule("implot"):
        implot_repository(name = "implot")
    if not native.existing_rule("com_github_mjbots_mjlib"):
        mjlib_repository(name = "com_github_mjbots_mjlib")
    if not native.existing_rule("moteus"):
        moteus_repository(name = "moteus")
    if not native.existing_rule("pi3hat"):
        pi3hat_repository(name = "pi3hat")
    if not native.existing_rule("rpi_bazel"):
        rpi_bazel_repository(name = "rpi_bazel")
    if not native.existing_rule("rules_pkg"):
        rules_pkg_repository(name = "rules_pkg")
    if not native.existing_rule("sophus"):
        sophus_repository(name = "sophus")
