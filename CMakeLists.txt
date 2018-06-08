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
add_subdirectory(astrodata)
add_subdirectory(doc)
# Set Data File path
SET(AstroData_DIR ${PROJECT_SOURCE_DIR}/astrodata)
# Set src file list
aux_source_directory (${PROJECT_SOURCE_DIR}/source/ SRC_LIST)

# Set Include h file path
list(APPEND SpaceDSL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)

# Set if build With OpenSSL or not
option(USE_OPENSSL "Build With OpenSSL." FALSE)
if (USE_OPENSSL)
    Add_Definitions(-DWITH_OPENSSL)
    find_package(OPENSSL)
    if(OPENSSL_FOUND)
        message(STATUS "#OPENSSL_INCLUDE_DIR = ${OPENSSL_INCLUDE_DIR}")
        list(APPEND SpaceDSL_INCLUDE_DIRS ${OPENSSL_INCLUDE_DIR})
    endif()
endif()


INCLUDE_DIRECTORIES(${SpaceDSL_INCLUDE_DIRS})
#message(STATUS "#CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
#
# Set the path to build projcet
#set(CMAKE_BINARY_DIR "${PROJECT_SOURCE_DIR}/Build")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/Build")
#set(EXECUTABLE_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/Build")
#set(PROJECT_BINARY_DIR  "${PROJECT_SOURCE_DIR}/Build")
#
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

message(STATUS "#CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
# Set if build SHARED LIBS  or not
option(BUILD_SHAREDLIBS "Build SHARED LIBS" FALSE)
if (BUILD_SHAREDLIBS)
    ADD_DEFINITIONS(-DSPACEDSL_SHARED_LIBRARY)
    # Set the build postfix extension according to what configuration is being built.
    IF (CMAKE_BUILD_TYPE MATCHES "Release")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
        ADD_LIBRARY(SpaceDSL SHARED ${SRC_LIST})
        SET_TARGET_PROPERTIES(SpaceDSL PROPERTIES VERSION 0.0 SOVERSION 0)
        SET(TargetName SpaceDSL)
    ELSEIF (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
        ADD_LIBRARY(SpaceDSL SHARED ${SRC_LIST})
        SET_TARGET_PROPERTIES(SpaceDSL PROPERTIES VERSION 0.0 SOVERSION 0)
        SET(TargetName SpaceDSL)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
        ADD_LIBRARY(SpaceDSL SHARED ${SRC_LIST})
        SET_TARGET_PROPERTIES(SpaceDSL PROPERTIES VERSION 0.0 SOVERSION 0)
        SET(TargetName SpaceDSL)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "Debug")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
        ADD_LIBRARY(SpaceDSLd SHARED ${SRC_LIST})
        SET_TARGET_PROPERTIES(SpaceDSLd PROPERTIES VERSION 0.0 SOVERSION 0)
        SET(TargetName SpaceDSLd)
    ELSE()
        SET(CMAKE_BUILD_POSTFIX "")
    ENDIF()
else()
    ADD_DEFINITIONS(-DSPACEDSL_STATIC_LIBRARY)
    # Set the build postfix extension according to what configuration is being built.
    IF (CMAKE_BUILD_TYPE MATCHES "Release")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
        ADD_LIBRARY(SpaceDSLs STATIC ${SRC_LIST})
        SET_TARGET_PROPERTIES(SpaceDSLs PROPERTIES VERSION 0.0 SOVERSION 0)
        SET(TargetName SpaceDSLs)
    ELSEIF (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
        ADD_LIBRARY(SpaceDSLs STATIC ${SRC_LIST})
        SET_TARGET_PROPERTIES(SpaceDSLs PROPERTIES VERSION 0.0 SOVERSION 0)
        SET(TargetName SpaceDSLs)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
        ADD_LIBRARY(SpaceDSLs STATIC ${SRC_LIST})
        SET_TARGET_PROPERTIES(SpaceDSLs PROPERTIES VERSION 0.0 SOVERSION 0)
        SET(TargetName SpaceDSLs)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "Debug")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
        ADD_LIBRARY(SpaceDSLsd STATIC ${SRC_LIST})
        SET_TARGET_PROPERTIES(SpaceDSLsd PROPERTIES VERSION 0.0 SOVERSION 0)
        SET(TargetName SpaceDSLsd)
    ELSE()
        SET(CMAKE_BUILD_POSTFIX "")
    ENDIF()
endif()

if(OPENSSL_FOUND)
    message(STATUS "#Target Link OPENSSL_LIBRARIES_DIR = ${OPENSSL_LIBRARIES}")
    target_link_libraries(${TargetName} ${OPENSSL_LIBRARIES})
endif()
#

# Set if build test file or not
option(BUILD_TESTS "Build Tests." TRUE)
if (BUILD_TESTS)
    enable_testing()
    add_subdirectory(ctest)
endif()

message(STATUS "#PROJECT_BINARY_DIR=${PROJECT_BINARY_DIR}")
FILE(COPY ${AstroData_DIR} DESTINATION ${PROJECT_BINARY_DIR})
#安装静态库和动态库
#INSTALL(TARGETS SpaceDSL   SpaceDSLs
#    LIBRARY DESTINATION lib
#    ARCHIVE DESTINATION lib)
#安装.h
#INSTALL(FILES hello.h DESTINATION include/hello)


