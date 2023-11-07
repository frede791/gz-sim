find_package(PkgConfig REQUIRED)
pkg_check_modules(GStreamer REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(Opencv REQUIRED IMPORTED_TARGET opencv4)


gz_add_system(gst-camera
  SOURCES
    GstCamera.cc

  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    PkgConfig::GStreamer
    PkgConfig::Opencv
)
