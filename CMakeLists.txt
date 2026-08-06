cmake_minimum_required(VERSION 3.20)
project(Rectilinear3dBoolean VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

# Sibling checkout dependency (relative path). Also brings in Eigen3 (found
# and linked PUBLIC by common_geometry's own CMakeLists.txt) transitively.
#
# Guarded with NOT TARGET: a consumer that pulls in this project alongside
# another sibling that also depends on common_geometry would otherwise
# add_subdirectory the same common_geometry path twice in one build tree,
# which CMake rejects. Whichever call runs first wins.
if(NOT TARGET common_geometry)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common_geometry
                    ${CMAKE_CURRENT_BINARY_DIR}/common_geometry)
endif()

add_library(rectilinear3d_boolean INTERFACE)

target_include_directories(rectilinear3d_boolean
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(rectilinear3d_boolean INTERFACE cxx_std_17)

# INTERFACE (not PRIVATE): boolean_op.hpp exposes ns_cg::BBox3d in public
# signatures, so consumers of rectilinear3d_boolean need common_geometry's
# (and transitively Eigen's) include paths too.
target_link_libraries(rectilinear3d_boolean INTERFACE common_geometry)

option(RECTILINEAR3D_BOOLEAN_BUILD_TESTS "Build tests" ON)
if(RECTILINEAR3D_BOOLEAN_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

option(RECTILINEAR3D_BOOLEAN_BUILD_EXAMPLES "Build example programs" ON)
if(RECTILINEAR3D_BOOLEAN_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
