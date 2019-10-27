cmake_minimum_required(VERSION 3.14)
project(mpl)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost 1.65 COMPONENTS program_options REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)

include_directories("${googlebenchmark_SOURCE_DIR}/include")
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories( ${Boost_INCLUDE_DIR} )
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../random_grid_map_generator)

include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(./include ./src)

link_directories(${OpenCV_LIBRARY_DIRS})
link_libraries(${PYTHON_LIBRARIES})

add_subdirectory(mpl_benchmark)
add_subdirectory(examples)
add_subdirectory(include)
add_subdirectory(src)
add_subdirectory(tests)