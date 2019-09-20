#************************************************#
#                                                #
# Copyright (C) 2017  Niu Zhiyong (China)        #
# <tbnzhyxxx@163.com>                            #
# Update:2018.03.20                              #
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
message(STATUS "#ALL_OUTPUT_PATH=${ALL_OUTPUT_PATH}")

# Set folder in project dir
add_subdirectory(include)
add_subdirectory(source)


# Set src file list
aux_source_directory (${PROJECT_SOURCE_DIR}/source/ SRC_LIST)
aux_source_directory (${PROJECT_SOURCE_DIR}/source/jpl SRC_LIST)
aux_source_directory (${PROJECT_SOURCE_DIR}/source/nrlmsise00 SRC_LIST)
aux_source_directory (${PROJECT_SOURCE_DIR}/source/gsoap SRC_LIST)
aux_source_directory (${PROJECT_SOURCE_DIR}/source/sxp SRC_LIST)

# Set Include h file path
list(APPEND SpaceDSL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)
list(APPEND SpaceDSL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/SpaceDSL)
list(APPEND SpaceDSL_INCLUDE_DIRS ../Dependence)

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

set(LIBRARY_OUTPUT_PATH  ${ALL_OUTPUT_PATH})

if (BUILD_SHARED_LIBS)
    ADD_DEFINITIONS(-DSPACEDSL_SHARED_LIBRARY)
        SET(TargetName SpaceDSL)
    # Set the build postfix extension according to what configuration is being built.
        ADD_LIBRARY(${TargetName} SHARED ${SRC_LIST})
        SET_TARGET_PROPERTIES(${TargetName} PROPERTIES VERSION 0.0 SOVERSION 0)
else()
    ADD_DEFINITIONS(-DSPACEDSL_STATIC_LIBRARY)
    SET(TargetName SpaceDSL_static)
        ADD_LIBRARY(${TargetName} STATIC ${SRC_LIST})
        SET_TARGET_PROPERTIES(${TargetName} PROPERTIES VERSION 0.0 SOVERSION 0)
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    message(STATUS "#CMAKE_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")
else()
    message(STATUS "#CMAKE_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")
    target_link_libraries(${TargetName} pthread)
endif()

if(OPENSSL_FOUND)
    message(STATUS "#Target Link OPENSSL_LIBRARIES_DIR = ${OPENSSL_LIBRARIES}")
    target_link_libraries(${TargetName} ${OPENSSL_LIBRARIES})
endif()
#


# Make Install File
INSTALL(TARGETS ${TargetName}
                RUNTIME DESTINATION ${ALL_OUTPUT_PATH}/Install
                LIBRARY DESTINATION ${ALL_OUTPUT_PATH}/Install
                ARCHIVE DESTINATION ${ALL_OUTPUT_PATH}/Install
                )
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION ${ALL_OUTPUT_PATH}/Install/Include)






