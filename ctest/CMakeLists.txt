#************************************************#
#                                                #
#       this cmakelist build for  ctesting       #
#                                                #
#************************************************#
cmake_minimum_required(VERSION 3.8.0)
set(CMAKE_CXX_STANDARD 14)

#include(CheckFunctionExists)
#include(CheckCXXSourceCompiles)
#include(CheckLibraryExists)
#include(CPack)
#enable_testing()

# Set Include h file path
list(APPEND CTEST_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/Dependence)
list(APPEND CTEST_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/SpaceDSL/include)
list(APPEND CTEST_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/SpaceDSL/include/SpaceDSL)
# Set Include h file patIh
list(APPEND CTEST_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/NLOpt/src/api)

# Set if build With OpenSSL or not
if (USE_OPENSSL)
    Add_Definitions(-DWITH_OPENSSL)
    find_package(OPENSSL)
    if(OPENSSL_FOUND)
        message(STATUS "#OPENSSL_INCLUDE_DIR = ${OPENSSL_INCLUDE_DIR}")
        list(APPEND CTEST_INCLUDE_DIRS ${OPENSSL_INCLUDE_DIR})
    endif()
endif()


INCLUDE_DIRECTORIES(${CTEST_INCLUDE_DIRS})
#
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

set(EXECUTABLE_OUTPUT_PATH  ${ALL_OUTPUT_PATH})
message(STATUS "#ALL_OUTPUT_PATH=${ALL_OUTPUT_PATH}")

LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/Build/")

# Set source file to make executable
add_executable(UnitTest TestMain.cpp)
add_executable(ThreadTest TestThread.cpp)
add_executable(NLOptTest TestNLOpt.cpp)

# UnitTest Set if build SHARED LIBS  or not
IF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    if (BUILD_SHARED_LIBS)
            TARGET_LINK_LIBRARIES(UnitTest SpaceDSL)
    else()
            ADD_DEFINITIONS(-DSPACEDSL_STATIC_LIBRARY)
            TARGET_LINK_LIBRARIES(UnitTest SpaceDSL_static)
    endif()
ELSE()
    if (BUILD_SHARED_LIBS)
            TARGET_LINK_LIBRARIES(UnitTest SpaceDSL pthread)
    else()
            TARGET_LINK_LIBRARIES(UnitTest SpaceDSL_static pthread)
    endif()
ENDIF()

# ThreadTest Set if build SHARED LIBS  or not
IF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    if (BUILD_SHARED_LIBS)
            TARGET_LINK_LIBRARIES(ThreadTest SpaceDSL)
    else()
            ADD_DEFINITIONS(-DSPACEDSL_STATIC_LIBRARY)
            TARGET_LINK_LIBRARIES(ThreadTest SpaceDSL_static)
    endif()
ELSE()
    if (BUILD_SHARED_LIBS)
            TARGET_LINK_LIBRARIES(ThreadTest SpaceDSL pthread)
    else()
            TARGET_LINK_LIBRARIES(ThreadTest SpaceDSL_static pthread)
    endif()
ENDIF()

# NLOptTest Set if build SHARED LIBS  or not
if (BUILD_SHARED_LIBS)
        TARGET_LINK_LIBRARIES(NLOptTest NLOpt)
else()

        TARGET_LINK_LIBRARIES(NLOptTest NLOpt_static)
endif()

if(OPENSSL_FOUND)
    message(STATUS "#Target Link OPENSSL_LIBRARIES_DIR = ${OPENSSL_LIBRARIES}")
    target_link_libraries(UnitTest ${OPENSSL_LIBRARIES})
endif()

if(OPENSSL_FOUND)
    message(STATUS "#Target Link OPENSSL_LIBRARIES_DIR = ${OPENSSL_LIBRARIES}")
    target_link_libraries(ThreadTest ${OPENSSL_LIBRARIES})
endif()

#add_test(UnitTest UnitTest)

# Make Install File

INSTALL(TARGETS UnitTest ThreadTest NLOptTest
                RUNTIME DESTINATION ${ALL_OUTPUT_PATH}/Install
                LIBRARY DESTINATION ${ALL_OUTPUT_PATH}/Install
                ARCHIVE DESTINATION ${ALL_OUTPUT_PATH}/Install
                )
