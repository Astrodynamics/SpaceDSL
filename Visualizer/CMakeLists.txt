#************************************************#
#                                                #
#       this cmakelist build for  ctesting       #
#                                                #
#************************************************#
cmake_minimum_required(VERSION 3.8.0)
set(CMAKE_CXX_STANDARD 14)

# Set project name
project(Visualizer)

# Set src file list
#aux_source_directory (${PROJECT_SOURCE_DIR}/source/ SRC_LIST)
list(APPEND SRC_LIST    source/main.cpp
                        source/GLStars.cpp
                        source/Rendering.cpp
                        source/OpenGLSupport.cpp)
# Set Include h file path
list(APPEND VISUALIZER_INCLUDE_DIRS ../Dependence)
list(APPEND VISUALIZER_INCLUDE_DIRS ../freeglut/include)
list(APPEND VISUALIZER_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)


INCLUDE_DIRECTORIES(${VISUALIZER_INCLUDE_DIRS})
#
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

set(EXECUTABLE_OUTPUT_PATH  ${ALL_OUTPUT_PATH})
message(STATUS "#ALL_OUTPUT_PATH=${ALL_OUTPUT_PATH}")

LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/Build/")

# Set source file to make executable
add_executable(Visualizer ${SRC_LIST})

# UnitTest Set if build SHARED LIBS  or not
if (BUILD_SHARED_LIBS)
        TARGET_LINK_LIBRARIES(Visualizer freeglut)
else()
        TARGET_LINK_LIBRARIES(Visualizer freeglut_static)
endif()

# Make Install File

INSTALL(TARGETS Visualizer 
                RUNTIME DESTINATION ${ALL_OUTPUT_PATH}/Install
                LIBRARY DESTINATION ${ALL_OUTPUT_PATH}/Install
                ARCHIVE DESTINATION ${ALL_OUTPUT_PATH}/Install
                )
