# lz4-config.cmake - Config file for lz4 library using pkg-config
# Provides lz4 library configuration for MCAP compression

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_LZ4 QUIET liblz4)
endif()

if(PC_LZ4_FOUND)
    set(lz4_FOUND TRUE)
    set(lz4_INCLUDE_DIRS ${PC_LZ4_INCLUDE_DIRS})
    set(lz4_LIBRARIES ${PC_LZ4_LIBRARIES})
    set(lz4_LIBRARY_DIRS ${PC_LZ4_LIBRARY_DIRS})
    
    if(NOT TARGET LZ4::lz4)
        add_library(LZ4::lz4 INTERFACE IMPORTED)
        set_target_properties(LZ4::lz4 PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${PC_LZ4_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${PC_LZ4_LIBRARIES}"
        )
    endif()
else()
    set(lz4_FOUND FALSE)
endif()
