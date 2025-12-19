# re2-config.cmake - Config file for re2 library using pkg-config
# This suppresses Arrow's "re2Config.cmake not found" warning

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_RE2 QUIET re2)
endif()

if(PC_RE2_FOUND)
    set(re2_FOUND TRUE)
    set(re2_INCLUDE_DIRS ${PC_RE2_INCLUDE_DIRS})
    set(re2_LIBRARIES ${PC_RE2_LIBRARIES})
    set(re2_LIBRARY_DIRS ${PC_RE2_LIBRARY_DIRS})
    
    if(NOT TARGET re2::re2)
        add_library(re2::re2 INTERFACE IMPORTED)
        set_target_properties(re2::re2 PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${PC_RE2_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${PC_RE2_LIBRARIES}"
        )
    endif()
else()
    set(re2_FOUND FALSE)
endif()
