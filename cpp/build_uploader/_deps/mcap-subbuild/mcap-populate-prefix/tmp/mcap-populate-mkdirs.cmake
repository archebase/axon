# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/zhexuany/Repo/Axon/cpp/build_uploader/_deps/mcap-src")
  file(MAKE_DIRECTORY "/Users/zhexuany/Repo/Axon/cpp/build_uploader/_deps/mcap-src")
endif()
file(MAKE_DIRECTORY
  "/Users/zhexuany/Repo/Axon/cpp/build_uploader/_deps/mcap-build"
  "/Users/zhexuany/Repo/Axon/cpp/build_uploader/_deps/mcap-subbuild/mcap-populate-prefix"
  "/Users/zhexuany/Repo/Axon/cpp/build_uploader/_deps/mcap-subbuild/mcap-populate-prefix/tmp"
  "/Users/zhexuany/Repo/Axon/cpp/build_uploader/_deps/mcap-subbuild/mcap-populate-prefix/src/mcap-populate-stamp"
  "/Users/zhexuany/Repo/Axon/cpp/build_uploader/_deps/mcap-subbuild/mcap-populate-prefix/src"
  "/Users/zhexuany/Repo/Axon/cpp/build_uploader/_deps/mcap-subbuild/mcap-populate-prefix/src/mcap-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/zhexuany/Repo/Axon/cpp/build_uploader/_deps/mcap-subbuild/mcap-populate-prefix/src/mcap-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/zhexuany/Repo/Axon/cpp/build_uploader/_deps/mcap-subbuild/mcap-populate-prefix/src/mcap-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
