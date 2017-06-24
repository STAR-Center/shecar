cmake_minimum_required(VERSION 2.8.3)
project(SHECAR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

file(GLOB SHECAR_INCLUDE
    src/*.h
    src/axxb/*.h
)
file(GLOB SHECAR_SRC
    src/*.cc
    src/*.cpp
    src/axxb/*.cc
)

find_package(Theia REQUIRED )

# Eigen
set(MIN_EIGEN_VERSION 3.2.0)
find_package(Eigen3 ${MIN_EIGEN_VERSION} REQUIRED)
if (EIGEN_FOUND)
  message("-- Found Eigen version ${EIGEN_VERSION}: ${EIGEN_INCLUDE_DIRS}")
else (CERES_FOUND)
  message(FATAL_ERROR "Can't find Eigen. Please set EIGEN_INCLUDE_DIR & "
    "EIGEN_LIBRARY")
endif (EIGEN_FOUND)

# Ceres
message("-- Check for Ceres")
find_package(Ceres REQUIRED SuiteSparse)
if (CERES_FOUND)
  message("-- Found Ceres: ${CERES_INCLUDE_DIRS}")
else (CERES_FOUND)
  message(FATAL_ERROR "Can't find Ceres. Please set CERES_INCLUDE_DIR & "
    "CERES_LIBRARY")
endif (CERES_FOUND)

include_directories(
  ${THEIA_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(SHECAR ${SHECAR_SRC} ${SHECAR_INCLUDE})
## Specify libraries to link a library or executable target against
target_link_libraries(SHECAR
  ${THEIA_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${CERES_LIBRARIES}
)

