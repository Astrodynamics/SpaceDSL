#************************************************#
#                                                #
#       this cmakelist build for  ctesting       #
#                                                #
#************************************************#
cmake_minimum_required(VERSION 3.8.0)

# Set project name
project(PySpaceDSL)

# Set  C++ standard version
set(CMAKE_CXX_STANDARD 14)

# Set src file list
aux_source_directory (${PROJECT_SOURCE_DIR}/../SpaceDSL/source/ SRC_LIST)
aux_source_directory (${PROJECT_SOURCE_DIR}/../SpaceDSL/source/jpl SRC_LIST)
aux_source_directory (${PROJECT_SOURCE_DIR}/../SpaceDSL/source/nrlmsise00 SRC_LIST)
aux_source_directory (${PROJECT_SOURCE_DIR}/../SpaceDSL/source/gsoap SRC_LIST)
aux_source_directory (${PROJECT_SOURCE_DIR}/../SpaceDSL/source/sxp SRC_LIST)

# Set Include h file path
list(APPEND PYLIB_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../Dependence)
list(APPEND PYLIB_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../SpaceDSL/include)
list(APPEND PYLIB_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../SpaceDSL/include/SpaceDSL)


# Set if build With Python or not
if (BUILD_PYTHON)
    find_package(PythonLibs REQUIRED)
    message(STATUS "#PYTHON_INCLUDE_DIR = ${PYTHON_INCLUDE_DIR}")
    list(APPEND PYLIB_INCLUDE_DIRS ${PYTHON_INCLUDE_DIR})
endif()

# Set if build With OpenSSL or not
if (USE_OPENSSL)
    Add_Definitions(-DWITH_OPENSSL)
    find_package(OPENSSL)
    if(OPENSSL_FOUND)
        message(STATUS "#OPENSSL_INCLUDE_DIR = ${OPENSSL_INCLUDE_DIR}")
        list(APPEND PYLIB_INCLUDE_DIRS ${OPENSSL_INCLUDE_DIR})
    endif()
endif()

INCLUDE_DIRECTORIES(${PYLIB_INCLUDE_DIRS})
#
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(LIBRARY_OUTPUT_PATH  ${ALL_OUTPUT_PATH})
message(STATUS "#ALL_OUTPUT_PATH=${ALL_OUTPUT_PATH}")


ADD_DEFINITIONS(-DSPACEDSL_SHARED_LIBRARY)
SET(TargetName PySpaceDSL)
# Set the build postfix extension according to what configuration is being built.
ADD_LIBRARY(${TargetName} SHARED ${SRC_LIST})

#target_link_libraries(${TargetName} PRIVATE pybind11::module)
SET_TARGET_PROPERTIES(${TargetName} PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                         SUFFIX "${PYTHON_MODULE_EXTENSION}")


if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    message(STATUS "#CMAKE_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")
else()
    message(STATUS "#CMAKE_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")
    target_link_libraries(${TargetName} pthread)
endif()


message(STATUS "#Target Link PYTHON_LIBRARIES_DIR = ${PYTHON_LIBRARIES}")
target_link_libraries(${TargetName} ${PYTHON_LIBRARIES})


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
#INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION ${ALL_OUTPUT_PATH}/Install/Include)


