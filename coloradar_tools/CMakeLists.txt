cmake_minimum_required(VERSION 3.12)
project(coloradar_tools VERSION 0.1 LANGUAGES C CXX)


# find_program(GCC_COMPILER NAMES gcc-12 gcc-11 gcc-10 gcc-9 gcc-8 gcc-7 PATHS /usr/bin NO_DEFAULT_PATH)
# if (NOT GCC_COMPILER)
#     message(FATAL_ERROR "No suitable GCC compiler found below version 13!")
# endif()
# string(REGEX REPLACE "gcc-[0-9]+" "g++" GPP_COMPILER ${GCC_COMPILER})
# set(CMAKE_C_COMPILER ${GCC_COMPILER} CACHE PATH "C compiler" FORCE)
# set(CMAKE_CXX_COMPILER ${GPP_COMPILER} CACHE PATH "C++ compiler" FORCE)
# message(STATUS "Using compiler: ${GCC_COMPILER}")

find_program(GCC_COMPILER NAMES gcc-12 gcc-11 gcc-10 PATHS /usr/bin NO_DEFAULT_PATH)
if (NOT GCC_COMPILER)
    message(FATAL_ERROR "No suitable GCC compiler found! Please ensure GCC version 10, 11, or 12 is installed.")
endif()
string(REGEX REPLACE "gcc-([0-9]+)" "g++-\\1" GPP_COMPILER ${GCC_COMPILER})
set(CMAKE_C_COMPILER ${GCC_COMPILER} CACHE PATH "C compiler" FORCE)
set(CMAKE_CXX_COMPILER ${GPP_COMPILER} CACHE PATH "C++ compiler" FORCE)
message(STATUS "Using GCC compiler: ${GCC_COMPILER}")
message(STATUS "Using G++ compiler: ${GPP_COMPILER}")


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-fconcepts)
# set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++20")
# add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
# add_definitions(-DBOOST_NO_CXX20_ALLOCATOR_TRAITS)
# add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED -DBOOST_ERROR_CODE_HEADER_ONLY)
# add_definitions(-DBOOST_NO_CXX20_ALLOCATOR)
# add_compile_options(-Wno-c++20-compat)
# add_definitions(-DBOOST_DISABLE_ASSERTS)


find_package(MPI REQUIRED)
# find_package(VTK COMPONENTS mpi REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL 1.10 REQUIRED)
# find_package(PCL 1.12 REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Boost 1.78 REQUIRED)
find_package(CUDA)
find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(pybind11)

set(CUDA_LIBS ${CUDA_LIBRARIES} ${CUDA_CUFFT_LIBRARIES})
# include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/src/include ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${HDF5_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})

### LIBS

# add_library(boost_target INTERFACE)
# target_compile_definitions(boost_target INTERFACE -DBOOST_NO_CXX20_ALLOCATOR)
# target_include_directories(boost_target SYSTEM INTERFACE ${Boost_INCLUDE_DIRS})



add_library(coloradar_tools SHARED
    src/libs/internal.cpp
    src/libs/configs.cpp
    src/libs/pcl_functions.cpp
    src/libs/octo_pointcloud.cpp
    src/libs/coloradar_run.cpp
    src/libs/dataset.cpp
)
target_link_libraries(coloradar_tools boost_target ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${HDF5_LIBRARIES} ${JSONCPP_LIBRARIES})


if(pybind11_FOUND)
    message(STATUS "Pybind11 found, compiling Python bindings.")
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    pybind11_add_module(coloradar_dataset_tools src/data_prep/python_bindings.cpp)
    target_link_libraries(coloradar_dataset_tools PRIVATE coloradar_tools)
else()
    message(WARNING "Pybind11 not found, skipping Python bindings compilation.")
endif()


if(CUDA_FOUND)
    message(STATUS "CUDA found, compiling CUDA code.")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++20")

    cuda_add_library(coloradar_cuda SHARED
        src/libs/cuda_kernels.cu
        src/libs/radar_processing.cu
    )
    target_link_libraries(coloradar_cuda coloradar_tools ${CUDA_LIBS})
    include_directories(${CUDA_INCLUDE_DIRS})

    if(pybind11_FOUND)
        pybind11_add_module(coloradar_cuda_tools src/data_prep/cuda_python_bindings.cpp)
        target_link_libraries(coloradar_cuda_tools PRIVATE coloradar_cuda)
    endif()

    # cuda_add_executable(compare_heatmaps src/data_prep/compare_heatmaps.cu)
    # target_link_libraries(compare_heatmaps coloradar_cuda)
else()
    message(STATUS "CUDA not found, skipping CUDA compilation.")
endif()


### TESTS

add_executable(coloradar_tests src/test/test_coloradar_tools.cpp)
target_link_libraries(coloradar_tests coloradar_tools gtest gtest_main pthread)


### SCRIPTS

# add_executable(test src/data_prep/test.cpp)
# target_link_libraries(test coloradar_tools)

# add_executable(interpolate_poses_for_lidar src/data_prep/interpolate_poses_for_lidar.cpp)
# target_link_libraries(interpolate_poses_for_lidar coloradar_tools)

# add_executable(interpolate_poses_for_radar src/data_prep/interpolate_poses_for_radar.cpp)
# target_link_libraries(interpolate_poses_for_radar coloradar_tools)

# add_executable(filter_cloud src/data_prep/filter_cloud.cpp)
# target_link_libraries(filter_cloud coloradar_tools)

# add_executable(build_octomap src/data_prep/build_octomap.cpp)
# target_link_libraries(build_octomap coloradar_tools)

# add_executable(sample_map_frames src/data_prep/sample_map_frames.cpp)
# target_link_libraries(sample_map_frames coloradar_tools)

# add_executable(create_cascade_pointclouds src/data_prep/create_cascade_pointclouds.cpp)
# target_link_libraries(create_cascade_pointclouds coloradar_tools)

# add_executable(make_dataset src/data_prep/make_dataset.cpp)
# target_link_libraries(make_dataset coloradar_tools)