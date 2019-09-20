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
set(ALL_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/Build" CACHE GLOBAL "ALL Project Output Path")
message(STATUS "#ALL_OUTPUT_PATH=${ALL_OUTPUT_PATH}")
# Set if build SHARED LIBS  or not
set(BUILD_SHARED_LIBS ON)

set(CMAKE_DEBUG_POSTFIX "_d")

# Set if build With OpenSSL or not
option(BUILD_PYTHON "Build Python Library." TRUE)

# Set if build With OpenSSL or not
option(USE_OPENSSL "Build With OpenSSL." FALSE)

# Set folder in project dir
add_subdirectory(astrodata)
add_subdirectory(doc)
add_subdirectory(Dependence)
add_subdirectory(SpaceDSL)
add_subdirectory(NLOpt)
add_subdirectory(ElectronVisualizer)
add_subdirectory(Python)

# Set Data File path
SET(AstroData_DIR ${PROJECT_SOURCE_DIR}/astrodata)
# Set Dependence path
SET(Dependence_DIR ${PROJECT_SOURCE_DIR}/Dependence)

# Set if build test file or not
option(BUILD_TESTS "Build Tests." TRUE)
if (BUILD_TESTS)
    #enable_testing()
    add_subdirectory(ctest)
endif()

FILE(COPY ${AstroData_DIR} DESTINATION ${ALL_OUTPUT_PATH})
FILE(COPY ${Dependence_DIR} DESTINATION ${ALL_OUTPUT_PATH})

# Make Install File
INSTALL(DIRECTORY ${AstroData_DIR} DESTINATION ${ALL_OUTPUT_PATH}/Install)
INSTALL(DIRECTORY ${AstroData_DIR} DESTINATION ${ALL_OUTPUT_PATH}/Install/Include)






