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
list(APPEND CTEST_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)
# Set Include h file path
list(APPEND CTEST_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/nlopt/src/api)

# Set if build With OpenSSL or not
option(USE_OPENSSL "Build With OpenSSL." TRUE)
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
add_executable(NLOptTest TestNLOpt.cpp)

# UnitTest Set if build SHARED LIBS  or not
if (BUILD_SHARED_LIBS)
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSL)
else()
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSLs)
endif()

# NLOptTest Set if build SHARED LIBS  or not
if (BUILD_SHARED_LIBS)
        TARGET_LINK_LIBRARIES(NLOptTest NLOpt)
else()

        TARGET_LINK_LIBRARIES(NLOptTest NLOpts)
endif()

if(OPENSSL_FOUND)
    message(STATUS "#Target Link OPENSSL_LIBRARIES_DIR = ${OPENSSL_LIBRARIES}")
    target_link_libraries(UnitTest ${OPENSSL_LIBRARIES})
endif()

#add_test(UnitTest UnitTest)

# Make Install File

INSTALL(TARGETS UnitTest NLOptTest
                RUNTIME DESTINATION ${ALL_OUTPUT_PATH}/Install
                LIBRARY DESTINATION ${ALL_OUTPUT_PATH}/Install
                ARCHIVE DESTINATION ${ALL_OUTPUT_PATH}/Install
                )
