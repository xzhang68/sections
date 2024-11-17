# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/xm/autonomy_ws/build/Open3D/open3d_downloads-prefix/src/open3d_downloads")
  file(MAKE_DIRECTORY "/home/xm/autonomy_ws/build/Open3D/open3d_downloads-prefix/src/open3d_downloads")
endif()
file(MAKE_DIRECTORY
  "/home/xm/autonomy_ws/build/Open3D/open3d_downloads-prefix/src/open3d_downloads-build"
  "/home/xm/autonomy_ws/build/Open3D/open3d_downloads-prefix"
  "/home/xm/autonomy_ws/build/Open3D/open3d_downloads-prefix/tmp"
  "/home/xm/autonomy_ws/build/Open3D/open3d_downloads-prefix/src/open3d_downloads-stamp"
  "/home/xm/autonomy_ws/build/Open3D/open3d_downloads-prefix/src"
  "/home/xm/autonomy_ws/build/Open3D/open3d_downloads-prefix/src/open3d_downloads-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/xm/autonomy_ws/build/Open3D/open3d_downloads-prefix/src/open3d_downloads-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/xm/autonomy_ws/build/Open3D/open3d_downloads-prefix/src/open3d_downloads-stamp${cfgdir}") # cfgdir has leading slash
endif()
