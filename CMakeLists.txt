#************************************************#
#                                                #
# Copyright (C) 2017  Niu Zhiyong (China)        #
# <tbnzhyxxx@163.com>                            #
# Update:2018.03.20                                                #
#************************************************#
cmake_minimum_required(VERSION 3.8.0)
message(STATUS "#CMAKE_VERSION=${CMAKE_VERSION}")
#
# Set project name
project(SpaceDSL)
#
# Set  C++ standard version
set(CMAKE_CXX_STANDARD 14)
#
# Set folder in project dir
add_subdirectory(include)
add_subdirectory(source)
# Set src file list
aux_source_directory (${PROJECT_SOURCE_DIR}/source/ SRC_LIST)

#
# Set Include h file path
set(SpaceDSL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
#INCLUDE_DIRECTORIES(include/ include/OXDeepNN)
INCLUDE_DIRECTORIES(${SpaceDSL_INCLUDE_DIR})
#message(STATUS "#OXDeepNN_INCLUDE_DIR=${OXDeepNN_INCLUDE_DIR}")
#message(STATUS "#CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
#
# Set the path to build projcet
set(CMAKE_BINARY_DIR "${PROJECT_SOURCE_DIR}/Build")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/Build")
set(EXECUTABLE_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/Build")
set(PROJECT_BINARY_DIR  "${PROJECT_SOURCE_DIR}/Build")
#
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Set source file to make executable
#add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/source/main.cpp )
#
ADD_DEFINITIONS(-DSPACEDSL_LIBRARY)

# Set the build postfix extension according to what configuration is being built.
IF (CMAKE_BUILD_TYPE MATCHES "Release")
    SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
    ADD_LIBRARY(SpaceDSL SHARED ${SRC_LIST})
    ADD_LIBRARY(SpaceDSLs STATIC ${SRC_LIST})
    SET_TARGET_PROPERTIES(SpaceDSL PROPERTIES VERSION 0.0 SOVERSION 0)
    SET_TARGET_PROPERTIES(SpaceDSLs PROPERTIES VERSION 0.0 SOVERSION 0)
ELSEIF (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
    SET(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
    ADD_LIBRARY(SpaceDSL SHARED ${SRC_LIST})
    ADD_LIBRARY(SpaceDSLs STATIC ${SRC_LIST})
    SET_TARGET_PROPERTIES(SpaceDSL PROPERTIES VERSION 0.0 SOVERSION 0)
    SET_TARGET_PROPERTIES(SpaceDSLs PROPERTIES VERSION 0.0 SOVERSION 0)
ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
    SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
    ADD_LIBRARY(SpaceDSL SHARED ${SRC_LIST})
    ADD_LIBRARY(SpaceDSLs STATIC ${SRC_LIST})
    SET_TARGET_PROPERTIES(SpaceDSL PROPERTIES VERSION 0.0 SOVERSION 0)
    SET_TARGET_PROPERTIES(SpaceDSLs PROPERTIES VERSION 0.0 SOVERSION 0)
ELSEIF(CMAKE_BUILD_TYPE MATCHES "Debug")
    SET(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
    ADD_LIBRARY(SpaceDSLd SHARED ${SRC_LIST})
    ADD_LIBRARY(SpaceDSLsd STATIC ${SRC_LIST})
    SET_TARGET_PROPERTIES(SpaceDSLd PROPERTIES VERSION 0.0 SOVERSION 0)
    SET_TARGET_PROPERTIES(SpaceDSLsd PROPERTIES VERSION 0.0 SOVERSION 0)
ELSE()
    SET(CMAKE_BUILD_POSTFIX "")
ENDIF()

# Set if build test file or not
option(BUILD_TESTS "Build Tests." TRUE)
if (BUILD_TESTS)
    enable_testing()
    add_subdirectory(ctest)
endif()

#安装静态库和动态库
#INSTALL(TARGETS SpaceDSL   SpaceDSLs
#    LIBRARY DESTINATION lib
#    ARCHIVE DESTINATION lib)
#安装.h
#INSTALL(FILES hello.h DESTINATION include/hello)


