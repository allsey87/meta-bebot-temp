SUMMARY = "AprilTag Library"
LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://src/apriltag.h;endline=30;md5=a5c93c14d66dce0ced23686283dce4ef"

inherit cmake

PR = "r0"
PV = "1.0"

SRC_URI = "file://src \
           file://CMakeLists.txt \
          "

S = "${WORKDIR}"
