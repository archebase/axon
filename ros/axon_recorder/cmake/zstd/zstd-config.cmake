# zstd-config.cmake - Config file for zstd library using pkg-config
# This suppresses Arrow's "zstdConfig.cmake not found" warning

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_ZSTD QUIET libzstd)
endif()

if(PC_ZSTD_FOUND)
    set(zstd_FOUND TRUE)
    set(zstd_INCLUDE_DIRS ${PC_ZSTD_INCLUDE_DIRS})
    set(zstd_LIBRARIES ${PC_ZSTD_LIBRARIES})
    set(zstd_LIBRARY_DIRS ${PC_ZSTD_LIBRARY_DIRS})
    
    if(NOT TARGET zstd::libzstd_shared)
        add_library(zstd::libzstd_shared INTERFACE IMPORTED)
        set_target_properties(zstd::libzstd_shared PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${PC_ZSTD_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${PC_ZSTD_LIBRARIES}"
        )
    endif()
else()
    set(zstd_FOUND FALSE)
endif()
