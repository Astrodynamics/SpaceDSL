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
set(SpaceDSL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${SpaceDSL_INCLUDE_DIR})
#
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

set(EXECUTABLE_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/Build")

LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/Build/")

# Set source file to make executable
add_executable(UnitTest TestMain.cpp)
# Set if build SHARED LIBS  or not
option(USE_SHAREDLIBS "USE SHARED LIBS" TRUE)
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
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSLd.lib)
    ELSEIF (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
       TARGET_LINK_LIBRARIES(UnitTest SpaceDSLd.lib)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSLd.lib)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "Debug")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
       TARGET_LINK_LIBRARIES(UnitTest SpaceDSLsd.lib)
    ELSE()
        SET(CMAKE_BUILD_POSTFIX "")
    ENDIF()
endif()



add_test(UnitTest UnitTest)

