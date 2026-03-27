# -*- python -*-


package(default_visibility = ["//visibility:public"])

load("//base:module_main.bzl", "module_main")
load("@rules_pkg//:pkg.bzl", "pkg_tar")


cc_library(
    name = "mech",
    srcs = [
        "mammal_ik.cc",
        "mime_type.cc",
        "nrfusb_client.cc",      
        "pi3hat_wrapper.cc",
        "quadruped.cc",
        "quadruped_control.cc",
        "quadruped_trot.cc",
        "rf_control.cc",       
        "system_info.cc",
        "swing_trajectory.cc",
        "trajectory.cc",
        "trajectory_line_intersect.cc",
        "web_server.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//base",
        "@boost//:filesystem",
        "@dart",
        "@com_github_mjbots_mjlib//mjlib/base:pid",
        "@com_github_mjbots_mjlib//mjlib/io:exclusive_command",
        "@com_github_mjbots_mjlib//mjlib/io:selector",
        "@com_github_mjbots_mjlib//mjlib/multiplex:asio_client",
        "@com_github_mjbots_mjlib//mjlib/multiplex:frame",
        "@com_github_mjbots_mjlib//mjlib/multiplex:register",
        "@sophus",
    ] + select({
        "//conditions:default" : [],
        "//:raspberrypi" : [
            "@pi3hat//lib/cpp/mjbots/pi3hat:libpi3hat",
        ],
    }),
    copts = [
        "-Wno-gnu-designator",
    ] + select({
        "//conditions:default" : [],
        "//:raspberrypi" : ["-DCOM_GITHUB_MJBOTS_RASPBERRYPI"],
    }),
    features = [
        "dbg",
    ],
)

cc_test(
    name = "test",
    srcs = ["test/" + x for x in [
        "expo_map_test.cc",
        "mammal_ik_test.cc",
        "swing_trajectory_test.cc",
        "trajectory_line_intersect_test.cc",
        "trajectory_test.cc",
        "test_main.cc",
        "vertical_line_frame_test.cc",
    ]],
    deps = [
        ":mech",
        "@boost//:test",
    ],
)

cc_binary(
    name = "direct_servo_latency_test",
    srcs = ["direct_servo_latency_test.cc"],
    deps = [":mech"],
)

module_main(
    name = "quadruped",
    cname = "mjmech::mech::Quadruped",
    prefix = "mech",
    deps = [":mech"],
)

cc_binary(
    name = "multiplex_tool",
    srcs = ["multiplex_tool_main.cc"],
    deps = [
        ":mech",
        "@com_github_mjbots_mjlib//mjlib/multiplex:libmultiplex_tool",
        "@org_llvm_libcxx//:libcxx",
    ],
)

pkg_tar(
    name = "tum_deploy",
    extension = "tar",
    include_runfiles = True,
    package_dir = "tum_deploy",
    srcs = [
        ":multiplex_tool",
        ":quadruped",
        "//utils:gui_assets",
    ],
    files = {
        "//configs:tum.ini": "configs/tum.ini",
        "//configs:tum.cfg": "configs/tum.cfg",
        "//utils:config_servos.py": "config_servos.py",
        "//utils:performance_governor.sh": "performance_governor.sh",
        "//utils:quad.cmd" : "quad.cmd",
        "//utils:quad_screen.conf" : "quad_screen.conf",
        "//utils:quad-start.sh" : "quad-start.sh",
        "//utils:start-robot.sh": "start-robot.sh",
        "//utils:zero_leg.py": "zero_leg.py",
        "//utils:zero_motors.py": "zero_motors.py",
        "//utils:tum_gui.py": "tum_gui.py",
    },
)

