# ===============================================================================
# Copyright (c) 2021 PTC Inc. All Rights Reserved.
#
# Confidential and Proprietary - Protected under copyright and other laws.
# Vuforia is a trademark of PTC Inc., registered in the United States and other
# countries.
# ===============================================================================
import argparse
import os
import sys
import subprocess
import pathlib
import configparser
from distutils.spawn import find_executable

VALID_ARCHS = ["armeabi-v7a", "arm64-v8a"]

CURRENT_DIR = os.path.abspath(pathlib.Path(__file__).parent.absolute())


def parse_args():
    """argument parser method

    Returns
    -------
    parser.parse_args
        the parsed arguments
    """

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-a",
        "--arch",
        help="Architectures. Space separated list of: armeabi-v7a, arm64-v8a",
        default=VALID_ARCHS,
        nargs="+",
    )

    parser.add_argument("-i", "--install", help="Install directory.", default=os.path.join("build"))

    parser.add_argument("-o", "--output", help="Location of generated build files.", default=os.path.join("build"))

    parser.add_argument(
        "-bt", "--build-type", help="Build type. Release or Debug", choices=["Release", "Debug"], default="Release"
    )

    parser.add_argument(
        "-vh",
        "--vuforia-header-dir",
        help="Directory that contains VuforiaEngine/Driver/Driver.h.",
        default=os.path.join(os.pardir, os.pardir, "build", "include"),
    )

    parser.add_argument(
        "-ue",
        "--uvc-external-dir",
        help="Directory containing libusb, libuvc and libjpeg-turbo build files. By default this expects the UVCCamera-repository layout and points to UVCCamera/libuvccamera/src/main/jni",
        default=os.path.join("UVCCamera", "libuvccamera", "src", "main", "jni"),
    )

    args = parser.parse_args()

    return args


def build_uvc_driver(
    architectures,
    build_type,
    output_dir,
    install_dir,
    vuforia_header_dir,
    uvc_external_dir,
):
    """Method for building the file driver

    Parameters
    ----------
    architecture : list
        list of the architectures
    build_type : str
        build type. Release or Debug
    output_dir : str
        path of the output directory
    install_dir : str
        path of the install directory
    vuforia_header_dir : str
        path which contians the vuforia headers
    """

    platform = "android"

    if architectures is None:
        architectures = VALID_ARCHS
        print("No architectures selected. using defaults: {0}".format(str(architectures)))

    for arch in architectures:
        if arch not in VALID_ARCHS:
            print("error: Invalid architecture: {0}".format(arch))
            sys.exit(1)

    output_rootdir = os.path.abspath(output_dir)
    install_rootdir = os.path.abspath(install_dir)
    vuforia_header_dir = os.path.abspath(vuforia_header_dir)
    android_toolchain_dir = os.path.join(CURRENT_DIR, "cmake")
    external_uvc_dir = os.path.abspath(uvc_external_dir)

    # If we're on windows fix up paths to remove backslashes that break cmake install
    if os.name == "nt":
        install_rootdir = install_rootdir.replace("\\", "/")
        vuforia_header_dir = vuforia_header_dir.replace("\\", "/")
        external_uvc_dir = external_uvc_dir.replace("\\", "/")

    if not os.path.exists(output_rootdir):
        os.makedirs(output_rootdir)

    # Check that cmake is installed.
    cmake_path = find_executable("cmake")
    if not cmake_path:
        print("error: Please install cmake and make sure that it is set in the PATH")
    else:
        print("Using cmake: {0}".format(cmake_path))

    # Check that ninja is installed for Android build.
    ninja_path = find_executable("ninja")
    if not ninja_path:
        print("error: Please install Ninja (https://ninja-build.org/) and make sure that it is set in the PATH")
    else:
        print("Using ninja: {0}".format(ninja_path))

    for arch in architectures:
        print("Generating arch: {0}".format(arch))
        output_dir = os.path.join(output_rootdir, platform, arch)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        toolchain_file = os.path.join(android_toolchain_dir, "android.toolchain.{0}.cmake".format(arch))
        generator = "Ninja"

        ret = subprocess.call(
            [
                cmake_path,
                "-G",
                generator,
                "-DVUFORIA_HEADER_DIR='{0}'".format(vuforia_header_dir),
                "-DEXTERNAL_UVC_DIR='{0}'".format(external_uvc_dir),
                "-DCMAKE_INSTALL_PREFIX='{0}'".format(install_rootdir),
                "-DCMAKE_TOOLCHAIN_FILE='{0}'".format(toolchain_file),
                "-DCMAKE_BUILD_TYPE={0}".format(build_type),
                CURRENT_DIR,
            ],
            cwd=output_dir,
        )

        if ret != 0:
            print("error: Project generation with cmake failed.")
            sys.exit(1)

        print("Building arch: {0}".format(arch))

        ret = subprocess.call([cmake_path, "--build", ".", "--target", "install"], cwd=output_dir)

        if ret != 0:
            print("error: Project generation with cmake failed.")
            sys.exit(1)

    print("Project generation and compilation succeeeded.")

def setup_environment(platform):
    """Optionally, a configuration file named environment.ini may be present to configure
    environment variables needed by the build.
    Example content:

    [ios]
    VARNAME1=VALUE1
    [macos]
    VARNAME1=VALUE2
    VARNAME2=VALUE3
    """
    env_file = os.path.join(CURRENT_DIR, "environment.ini")
    if os.path.exists(env_file):
        cfg = configparser.ConfigParser()
        cfg.optionxform = lambda option: option   # retain case of keys
        cfg.read(env_file)
        if cfg.has_section(platform):
            for key, value in cfg.items(platform):
                os.environ[key] = value
                print ("Set environment variable '{0}' to '{1}".format(key, value))

def main():
    """The build.py file's main method"""

    parsed_args = parse_args()
    setup_environment("android")

    build_uvc_driver(
        parsed_args.arch,
        parsed_args.build_type,
        parsed_args.output,
        parsed_args.install,
        parsed_args.vuforia_header_dir,
        parsed_args.uvc_external_dir,
    )


if __name__ == "__main__":
    main()
