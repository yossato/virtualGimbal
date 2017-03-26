cmake_minimum_required (VERSION 3.0)
project (biCubic)

find_package(OpenGL REQUIRED)
#OPENCV
find_package(OpenCV REQUIRED )
#OpenGL and GLFW
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW 3.2 REQUIRED glfw3)

set(ALL_LIBS
	${OPENGL_LIBRARY}
        ${OpenCV_LIBS}
        ${GLFW_LIBRARIES}
        GLEW
)
include_directories(include ${GLFW_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
add_executable(biCubic src/main.cpp src/shader.cpp src/texture.cpp src/controls.cpp)
target_link_libraries(biCubic ${ALL_LIBS}  -lGL -ldl -lXrandr -lXi -lXrender -ldrm -lXdamage -lXxf86vm -lXext -lX11 -lpthread -lXinerama -lXcursor -lrt)

#~ # デバッグビルドする場合は以下3行のコメントを解除する
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS_DEBUG "-O0 -g")
add_definitions(-DDEBUG)

#~ # リリースビルドする場合は以下の2行のコメントを解除する
#~ set(CMAKE_BUILD_TYPE Release)
#~ set(CMAKE_C_FLAGS_RELEASE "-O3 -lm")


#以下は変更しない
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -lm -L/usr/lib -lpthread -pthread -march=native -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lm -L/usr/lib -lpthread -pthread -march=native -std=c++0x -pthread")
