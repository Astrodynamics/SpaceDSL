#************************************************#
#                                                #
#       this cmakelist build for  ctesting       #
#                                                #
#************************************************#
cmake_minimum_required(VERSION 3.8.0)
set(CMAKE_CXX_STANDARD 14)

include(CheckFunctionExists)
include(CheckCXXSourceCompiles)
include(CheckLibraryExists)
include(CPack)
enable_testing()

# Set Include h file path
# Set Include h file path
list(APPEND CTEST_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)

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

set(EXECUTABLE_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/Build")

LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/Build/")

message(STATUS "#CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
# Set source file to make executable
add_executable(UnitTest TestMain.cpp)

# Set if build SHARED LIBS  or not
option(USE_SHAREDLIBS "USE SHARED LIBS" FALSE)
if (USE_SHAREDLIBS)
    ADD_DEFINITIONS(-DSPACEDSL_SHARED_LIBRARY)
    IF (CMAKE_BUILD_TYPE MATCHES "Release")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSL.lib)
    ELSEIF (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
       TARGET_LINK_LIBRARIES(UnitTest SpaceDSL.lib)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSL.lib)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "Debug")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
       TARGET_LINK_LIBRARIES(UnitTest SpaceDSLd.lib)
    ELSE()
        SET(CMAKE_BUILD_POSTFIX "")
    ENDIF()
else()
    ADD_DEFINITIONS(-DSPACEDSL_STATIC_LIBRARY)
    IF (CMAKE_BUILD_TYPE MATCHES "Release")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSLs.lib)
    ELSEIF (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
       TARGET_LINK_LIBRARIES(UnitTest SpaceDSLs.lib)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSLs.lib)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "Debug")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
       TARGET_LINK_LIBRARIES(UnitTest SpaceDSLsd.lib)
    ELSE()
        SET(CMAKE_BUILD_POSTFIX "")
    ENDIF()
endif()

if(OPENSSL_FOUND)
    message(STATUS "#Target Link OPENSSL_LIBRARIES_DIR = ${OPENSSL_LIBRARIES}")
    target_link_libraries(UnitTest ${OPENSSL_LIBRARIES})
endif()

add_test(UnitTest UnitTest)

