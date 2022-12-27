# CMake entry point
cmake_minimum_required (VERSION 3.0)
project (virtualGimbal)

find_package(OpenCV 4 REQUIRED 
NO_MODULE 
PATHS /usr/local 
NO_DEFAULT_PATH)
set(Python_ADDITIONAL_VERSIONS 3.5)

find_package(Eigen3 REQUIRED)


# Copy setting files
configure_file(chess_board_settings.ini chess_board_settings.ini)
configure_file(camera_descriptions/cameras_example.json camera_descriptions/cameras_example.json )
configure_file(scripts/read_angular_velocity_from_virtualGimbal.py scripts/read_angular_velocity_from_virtualGimbal.py)
configure_file(scripts/generate_angular_velocity_json.py scripts/generate_angular_velocity_json.py)
configure_file(scripts/setup_udev_rules.sh scripts/setup_udev_rules.sh)
configure_file(scripts/erase_flash_memory.py scripts/erase_flash_memory.py)
configure_file(records/sample.json records/sample.json)
configure_file(cl/stabilizer_kernel.cl cl/stabilizer_kernel.cl)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PYTHON_INCLUDE_DIRS})
set(ALL_LIBS
        ${OpenCV_LIBS}
)

add_executable(camera_calibration src/camera_calibration.cpp
    src/mINIRead.cpp
    src/json_tools.cpp
    src/camera_information.cpp)
target_link_libraries(camera_calibration ${ALL_LIBS} ${OpenCV_LIBS})

add_executable(pixelwise_stabilizer src/main.cpp
        src/virtual_gimbal_manager.cpp
        src/json_tools.cpp
        src/camera_information.cpp
        src/rotation_param.cpp
        src/calcShift.cpp
        src/distortion.cpp
        src/SO3Filters.cpp
        src/cl_manager.cpp
        src/multi_thread_video_writer.cpp
        src/inpainting.cpp
        src/data_collection.cpp
        src/point_pairs.cpp
        src/ransac.cpp
)
target_link_libraries(pixelwise_stabilizer ${ALL_LIBS} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})

add_executable(angular_velocity_estimator src/angular_velocity_estimator.cpp
        src/json_tools.cpp
        src/calcShift.cpp
        src/camera_information.cpp
)
target_link_libraries(angular_velocity_estimator ${ALL_LIBS} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})

add_executable(camera_pose_analyzer src/camera_pose_analyzer.cpp
        src/inpainting.cpp src/stabilizer_kernel_cpu.cpp)
target_link_libraries(camera_pose_analyzer ${ALL_LIBS} ${OpenCV_LIBS})


# Debug build
IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
set(CMAKE_C_FLAGS_DEBUG "-O0 -g")
ENDIF(CMAKE_BUILD_TYPE MATCHES DEBUG)

# Release build
IF(CMAKE_BUILD_TYPE MATCHES RELEASE)
    set(CMAKE_BUILD_TYPE Release)
    set(CMAKE_C_FLAGS_RELEASE "-O3 -lm")
    add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_MPL2_ONLY)
ENDIF(CMAKE_BUILD_TYPE MATCHES RELEASE)

#gprof使う場合は以下２行の最後に-pgをつけると良いみたい
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -lm -L/usr/lib -lpthread -pthread -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lm -L/usr/lib -lpthread -pthread -std=gnu++11 -pthread -Wall")
