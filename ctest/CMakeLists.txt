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

message(STATUS "#CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
# Set source file to make executable
add_executable(UnitTest TestMain.cpp)
add_executable(NLOptTest TestNLOpt.cpp)

# UnitTest Set if build SHARED LIBS  or not
if (BUILD_SHARED_LIBS)
    IF (CMAKE_BUILD_TYPE MATCHES "Release")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSL)
    ELSEIF (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
       TARGET_LINK_LIBRARIES(UnitTest SpaceDSL)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSL)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "Debug")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
       TARGET_LINK_LIBRARIES(UnitTest SpaceDSLd)
    ELSE()
        SET(CMAKE_BUILD_POSTFIX "")
    ENDIF()
else()
    IF (CMAKE_BUILD_TYPE MATCHES "Release")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSLs)
    ELSEIF (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
       TARGET_LINK_LIBRARIES(UnitTest SpaceDSLs)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
        TARGET_LINK_LIBRARIES(UnitTest SpaceDSLs)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "Debug")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
       TARGET_LINK_LIBRARIES(UnitTest SpaceDSLsd)
    ELSE()
        SET(CMAKE_BUILD_POSTFIX "")
    ENDIF()
endif()

# NLOptTest Set if build SHARED LIBS  or not
if (BUILD_SHARED_LIBS)
    IF (CMAKE_BUILD_TYPE MATCHES "Release")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
        TARGET_LINK_LIBRARIES(NLOptTest NLOpt)
    ELSEIF (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
       TARGET_LINK_LIBRARIES(NLOptTest NLOpt)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
        TARGET_LINK_LIBRARIES(NLOptTest NLOpt)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "Debug")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
       TARGET_LINK_LIBRARIES(NLOptTest NLOptd)
    ELSE()
        SET(CMAKE_BUILD_POSTFIX "")
    ENDIF()
else()
    IF (CMAKE_BUILD_TYPE MATCHES "Release")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
        TARGET_LINK_LIBRARIES(NLOptTest NLOpts)
    ELSEIF (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
       TARGET_LINK_LIBRARIES(NLOptTest NLOpts)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
        TARGET_LINK_LIBRARIES(NLOptTest NLOpts)
    ELSEIF(CMAKE_BUILD_TYPE MATCHES "Debug")
        SET(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
       TARGET_LINK_LIBRARIES(NLOptTest NLOptsd)
    ELSE()
        SET(CMAKE_BUILD_POSTFIX "")
    ENDIF()
endif()

if(OPENSSL_FOUND)
    message(STATUS "#Target Link OPENSSL_LIBRARIES_DIR = ${OPENSSL_LIBRARIES}")
    target_link_libraries(UnitTest ${OPENSSL_LIBRARIES})
endif()

add_test(UnitTest UnitTest)

