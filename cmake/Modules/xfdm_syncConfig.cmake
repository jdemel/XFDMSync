INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_XFDM_SYNC xfdm_sync)

FIND_PATH(
    XFDM_SYNC_INCLUDE_DIRS
    NAMES xfdm_sync/api.h
    HINTS $ENV{XFDM_SYNC_DIR}/include
        ${PC_XFDM_SYNC_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    XFDM_SYNC_LIBRARIES
    NAMES gnuradio-xfdm_sync
    HINTS $ENV{XFDM_SYNC_DIR}/lib
        ${PC_XFDM_SYNC_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(XFDM_SYNC DEFAULT_MSG XFDM_SYNC_LIBRARIES XFDM_SYNC_INCLUDE_DIRS)
MARK_AS_ADVANCED(XFDM_SYNC_LIBRARIES XFDM_SYNC_INCLUDE_DIRS)

