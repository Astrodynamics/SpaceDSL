#==============================================================================
# NLOPT 2.5 (2018) CMake file
#
# NLopt is a free/open-source library for nonlinear optimization, providing
# a common interface for a number of different free optimization routines
# available online as well as original implementations of various other
# algorithms
# WEBSITE: http://ab-initio.mit.edu/wiki/index.php/NLopt
# AUTHOR: Steven G. Johnson
#
# This CMakeLists.txt file was created to compile NLOPT with the CMAKE utility.
# Benoit Scherrer, 2010 CRL, Harvard Medical School
# Copyright (c) 2008-2009 Children's Hospital Boston
#==============================================================================
cmake_minimum_required(VERSION 3.8.0)

project (NLOpt)

add_subdirectory (src)
add_subdirectory (src/api)
add_subdirectory (src/algs)
add_subdirectory (src/util)

include (CheckIncludeFiles)
include (CheckFunctionExists)
include (CheckTypeSize)
include (CheckCCompilerFlag)
include (CheckCXXSymbolExists)
include (CheckCXXCompilerFlag)
include (CheckLibraryExists)

#==============================================================================
# COMPILATION CHECKINGS and CONFIGURATION GENERATION
#==============================================================================
check_include_file (dlfcn.h HAVE_DLFCN_H)
check_include_file (getopt.h HAVE_GETOPT_H)
check_include_file (unistd.h HAVE_UNISTD_H)
check_include_file (string.h HAVE_STRING_H)
check_include_file (strings.h HAVE_STRINGS_H)
check_include_file (inttypes.h HAVE_INTTYPES_H)
check_include_file (memory.h HAVE_MEMORY_H)
check_include_file (stdlib.h HAVE_STDLIB_H)
check_include_file (stdint.h HAVE_STDINT_H)
check_include_file (time.h HAVE_TIME_H)
check_include_file (sys/types.h HAVE_SYS_TYPES_H)
check_include_file (sys/stat.h HAVE_SYS_STAT_H)
check_include_file (sys/time.h HAVE_SYS_TIME_H)
if (HAVE_TIME_H AND HAVE_SYS_TIME_H)
  set (TIME_WITH_SYS_TIME TRUE)
endif ()
check_function_exists (getpid HAVE_GETPID)
check_function_exists (syscall HAVE_GETTID_SYSCALL)
check_function_exists (isinf HAVE_ISINF)
check_function_exists (isnan HAVE_ISNAN)
check_function_exists (gettimeofday HAVE_GETTIMEOFDAY)
check_function_exists (qsort_r HAVE_QSORT_R)
check_function_exists (time HAVE_TIME)
check_function_exists (copysign HAVE_COPYSIGN)
check_type_size ("uint32_t" SIZEOF_UINT32_T)
set (HAVE_UINT32_T ${SIZEOF_UINT32_T})
check_type_size ("unsigned int" SIZEOF_UNSIGNED_INT)
check_type_size ("unsigned long" SIZEOF_UNSIGNED_LONG)

#==============================================================================
# version
#==============================================================================
set (NLOPT_MAJOR_VERSION "2")
set (NLOPT_MINOR_VERSION "5")
set (NLOPT_BUGFIX_VERSION "0")
set (NLOPT_VERSION_STRING ${NLOPT_MAJOR_VERSION}.${NLOPT_MINOR_VERSION}.${NLOPT_BUGFIX_VERSION})
message (STATUS "NLopt version ${NLOPT_VERSION_STRING}")

#==============================================================================
# CREATE nlopt_config.h
#==============================================================================

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/nlopt_config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/nlopt_config.h IMMEDIATE)

#==============================================================================
# nlopt LIBRARY TARGET (SHARED OR STATIC)
#==============================================================================

#configure_file (src/api/nlopt.h ${PROJECT_BINARY_DIR}/include/NLOpt/nlopt.h IMMEDIATE)
#configure_file (src/api/nlopt.hpp ${PROJECT_BINARY_DIR}/include/NLOpt/nlopt.hpp IMMEDIATE)

set (NLOPT_HEADERS ${PROJECT_BINARY_DIR}/src/api/nlopt.h ${PROJECT_BINARY_DIR}/src/api/nlopt.hpp)

set (NLOPT_SOURCES
  src/algs/ags/data_types.hpp src/algs/ags/evolvent.hpp src/algs/ags/evolvent.cc src/algs/ags/solver.hpp src/algs/ags/solver.cc
  src/algs/ags/local_optimizer.hpp src/algs/ags/local_optimizer.cc src/algs/ags/ags.h src/algs/ags/ags.cc
  src/algs/direct/DIRect.c src/algs/direct/direct_wrap.c src/algs/direct/DIRserial.c src/algs/direct/DIRsubrout.c src/algs/direct/direct-internal.h src/algs/direct/direct.h
  src/algs/cdirect/cdirect.c src/algs/cdirect/hybrid.c src/algs/cdirect/cdirect.h
  src/algs/praxis/praxis.c src/algs/praxis/praxis.h
  src/algs/luksan/plis.c src/algs/luksan/plip.c src/algs/luksan/pnet.c src/algs/luksan/mssubs.c src/algs/luksan/pssubs.c src/algs/luksan/luksan.h
  src/algs/crs/crs.c src/algs/crs/crs.h
  src/algs/mlsl/mlsl.c src/algs/mlsl/mlsl.h
  src/algs/mma/mma.c src/algs/mma/mma.h src/algs/mma/ccsa_quadratic.c
  src/algs/cobyla/cobyla.c src/algs/cobyla/cobyla.h
  src/algs/newuoa/newuoa.c src/algs/newuoa/newuoa.h
  src/algs/neldermead/nldrmd.c src/algs/neldermead/neldermead.h src/algs/neldermead/sbplx.c
  src/algs/auglag/auglag.c src/algs/auglag/auglag.h
  src/algs/bobyqa/bobyqa.c src/algs/bobyqa/bobyqa.h
  src/algs/isres/isres.c src/algs/isres/isres.h
  src/algs/slsqp/slsqp.c src/algs/slsqp/slsqp.h
  src/algs/esch/esch.c src/algs/esch/esch.h
  src/api/general.c src/api/options.c src/api/optimize.c src/api/deprecated.c src/api/nlopt-internal.h src/api/nlopt.h src/api/f77api.c src/api/f77funcs.h src/api/f77funcs_.h src/api/nlopt.hpp
  src/util/mt19937ar.c src/util/sobolseq.c src/util/soboldata.h src/util/timer.c src/util/stop.c src/util/nlopt-util.h src/util/redblack.c src/util/redblack.h src/util/qsort_r.c src/util/rescale.c
)

list (APPEND NLOPT_SOURCES src/algs/stogo/global.cc src/algs/stogo/linalg.cc src/algs/stogo/local.cc src/algs/stogo/stogo.cc src/algs/stogo/tools.cc
      src/algs/stogo/global.h src/algs/stogo/linalg.h src/algs/stogo/local.h src/algs/stogo/stogo_config.h src/algs/stogo/stogo.h src/algs/stogo/tools.h)


#==============================================================================
# INCLUDE DIRECTORIES
#==============================================================================
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

INCLUDE_DIRECTORIES (
                    ${PROJECT_SOURCE_DIR}/src/algs/stogo
                    ${PROJECT_SOURCE_DIR}/src/algs/ags
                    ${PROJECT_SOURCE_DIR}/src/util
                    ${PROJECT_SOURCE_DIR}/src/algs/direct
                    ${PROJECT_SOURCE_DIR}/src/algs/cdirect
                    ${PROJECT_SOURCE_DIR}/src/algs/praxis
                    ${PROJECT_SOURCE_DIR}/src/algs/luksan
                    ${PROJECT_SOURCE_DIR}/src/algs/crs
                    ${PROJECT_SOURCE_DIR}/src/algs/mlsl
                    ${PROJECT_SOURCE_DIR}/src/algs/mma
                    ${PROJECT_SOURCE_DIR}/src/algs/cobyla
                    ${PROJECT_SOURCE_DIR}/src/algs/newuoa
                    ${PROJECT_SOURCE_DIR}/src/algs/neldermead
                    ${PROJECT_SOURCE_DIR}/src/algs/auglag
                    ${PROJECT_SOURCE_DIR}/src/algs/bobyqa
                    ${PROJECT_SOURCE_DIR}/src/algs/isres
                    ${PROJECT_SOURCE_DIR}/src/algs/slsqp
                    ${PROJECT_SOURCE_DIR}/src/algs/esch
                    ${PROJECT_SOURCE_DIR}/src/api)

set (NLOpt_LIB NLOpt)

set(LIBRARY_OUTPUT_PATH  ${ALL_OUTPUT_PATH})
message(STATUS "#ALL_OUTPUT_PATH=${ALL_OUTPUT_PATH}")

# Set if build SHARED LIBS  or not
if (BUILD_SHARED_LIBS)
    ADD_DEFINITIONS (-DNLOPT_DLL)
    ADD_DEFINITIONS (-DNLOPT_DLL_EXPORT)
    SET(TargetName ${NLOpt_LIB})
    ADD_LIBRARY(${TargetName} SHARED ${NLOPT_SOURCES})
else()
    # Set the build postfix extension according to what configuration is being built.
    SET(TargetName ${NLOpt_LIB}_static)
    ADD_LIBRARY(${TargetName} STATIC ${NLOPT_SOURCES})
endif()

# Make Install File

INSTALL(TARGETS ${TargetName}
                RUNTIME DESTINATION ${ALL_OUTPUT_PATH}/Install
                LIBRARY DESTINATION ${ALL_OUTPUT_PATH}/Install
                ARCHIVE DESTINATION ${ALL_OUTPUT_PATH}/Install
                )
INSTALL(FILES src/api/nlopt.h src/api/nlopt.hpp
                DESTINATION ${ALL_OUTPUT_PATH}/Install/Include/NLOpt)

