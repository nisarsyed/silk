include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(silk_package_dir "${CMAKE_INSTALL_LIBDIR}/cmake/silk")
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/silkConfig.cmake.in"
    "${PROJECT_BINARY_DIR}/silkConfig.cmake"
    INSTALL_DESTINATION "${silk_package_dir}"
)
# Source compatibility is evolving; no binary ABI promise is made. A requested
# package version must match this release rather than accepting a newer minor.
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/silkConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY ExactVersion
)
install(TARGETS silk EXPORT silkTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/silk"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING PATTERN "*.h"
)
install(EXPORT silkTargets NAMESPACE silk::
    DESTINATION "${silk_package_dir}"
)
install(FILES
    "${PROJECT_BINARY_DIR}/silkConfig.cmake"
    "${PROJECT_BINARY_DIR}/silkConfigVersion.cmake"
    DESTINATION "${silk_package_dir}"
)
