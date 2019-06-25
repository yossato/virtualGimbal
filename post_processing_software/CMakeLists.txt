# CMake entry point
cmake_minimum_required (VERSION 3.0)
project (virtualGimbal)

find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED )
set(Python_ADDITIONAL_VERSIONS 3.5)
find_package(PythonInterp 3.5 REQUIRED)
find_package(PythonLibs 3.5 REQUIRED)

if( CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR )
    message( FATAL_ERROR "Please select another Build Directory ! (and give it a clever name, like bin_Visual2012_64bits/)" )
endif()
if( CMAKE_SOURCE_DIR MATCHES " " )
        message( "Your Source Directory contains spaces. If you experience problems when compiling, this can be the cause." )
endif()
if( CMAKE_BINARY_DIR MATCHES " " )
        message( "Your Build Directory contains spaces. If you experience problems when compiling, this can be the cause." )
endif()

# Copy setting files
configure_file(chess_board_settings.ini chess_board_settings.ini)
configure_file(TextureFragmentShader.fragmentshader TextureFragmentShader.fragmentshader)
configure_file(TransformVertexShader.vertexshader TransformVertexShader.vertexshader)
configure_file(camera_descriptions/cameras_example.json camera_descriptions/cameras_example.json )
configure_file(scripts/read_angular_velocity_from_virtualGimbal.py scripts/read_angular_velocity_from_virtualGimbal.py)
configure_file(scripts/generate_angular_velocity_json.py scripts/generate_angular_velocity_json.py)
configure_file(scripts/setup_udev_rules.sh scripts/setup_udev_rules.sh)
configure_file(scripts/erase_flash_memory.py scripts/erase_flash_memory.py)
configure_file(records/sample.json records/sample.json)

# Compile external dependencies
add_subdirectory (external)

# On Visual 2005 and above, this module can set the debug working directory
cmake_policy(SET CMP0026 OLD)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/rpavlik-cmake-modules-fe2273")
include(CreateLaunchers)
include(MSVCMultipleProcessCompile) # /MP

if(INCLUDE_DISTRIB)
        add_subdirectory(distrib)
endif(INCLUDE_DISTRIB)



include_directories(
        external/AntTweakBar-1.16/include/
        external/glfw-3.1.2/include/GLFW/
        external/glm-0.9.7.1/
        external/glew-1.13.0/include/
        external/assimp-3.0.1270/include/
        external/bullet-2.81-rev2613/src/
        .
)
include_directories(include)
include_directories(/usr/local/include/eigen3)
include_directories(${PYTHON_INCLUDE_DIRS})
set(ALL_LIBS
        ${OPENGL_LIBRARY}
        ${OpenCV_LIBS}
        glfw
        GLEW_1130
)

add_definitions(
        -DTW_STATIC
        -DTW_NO_LIB_PRAGMA
        -DTW_NO_DIRECT3D
        -DGLEW_STATIC
        -D_CRT_SECURE_NO_WARNINGS
        # -DRAPIDJSON_HAS_STDSTRING
)

add_executable(virtualGimbal src/mainsyncandstabilize.cpp
        common/shader.cpp
        common/shader.hpp
        common/controls.cpp
        common/controls.hpp
        common/texture.cpp
        common/texture.hpp

        src/visualizer.cpp
        src/vsp.cpp
        src/calcShift.cpp
        src/mINIRead.cpp
        src/seekablevideocapture.cpp
        src/stabilize.cpp
        src/distortion.cpp
        src/visualizer.cpp
        src/json_tools.cpp
        src/camera_information.cpp
        include/settings.h
        include/distortion.h
        include/visualizer.h
        include/vsp.h
        include/camera_information.h

        TransformVertexShader.vertexshader
        TextureFragmentShader.fragmentshader
)
target_link_libraries(virtualGimbal ${ALL_LIBS} ${PYTHON_LIBRARIES})

add_executable(camera_calibration src/camera_calibration.cpp
    src/mINIRead.cpp
    src/json_tools.cpp
    src/camera_information.cpp)
target_link_libraries(camera_calibration ${ALL_LIBS} ${OpenCV_LIBS})

add_executable(rolling_shutter_parameter_estimator src/rolling_shutter_parameter_estimator.cpp
        src/mINIRead.cpp
        src/json_tools.cpp
        src/camera_information.cpp
        src/virtual_gimbal_manager.cpp
        src/rotation_param.cpp
        src/calcShift.cpp
        src/visualizer.cpp
        src/rotation_math.cpp
        src/distortion.cpp
        src/SO3Filters.cpp
        )
target_link_libraries(rolling_shutter_parameter_estimator ${ALL_LIBS} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})

add_executable(pixelwise_stabilizer src/main.cpp
        src/virtual_gimbal_manager.cpp
        src/json_tools.cpp
        src/camera_information.cpp
        src/rotation_param.cpp
        src/calcShift.cpp
        src/distortion.cpp
        src/SO3Filters.cpp
        src/visualizer.cpp #デバッグ専用。後で消す。
)
target_link_libraries(pixelwise_stabilizer ${ALL_LIBS} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})

# デバッグビルド
IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
set(CMAKE_C_FLAGS_DEBUG "-O0 -g")
ENDIF(CMAKE_BUILD_TYPE MATCHES DEBUG)

# リリースビルド
IF(CMAKE_BUILD_TYPE MATCHES RELEASE)
    set(CMAKE_BUILD_TYPE Release)
    set(CMAKE_C_FLAGS_RELEASE "-O3 -lm")
    add_definitions(-DEIGEN_NO_DEBUG)
ENDIF(CMAKE_BUILD_TYPE MATCHES RELEASE)

#gprof使う場合は以下２行の最後に-pgをつけると良いみたい
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -lm -L/usr/lib -lpthread -pthread -march=native -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lm -L/usr/lib -lpthread -pthread -march=native -std=c++11 -pthread -Wall")
