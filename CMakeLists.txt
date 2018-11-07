cmake_minimum_required(VERSION 3.1)

IF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  MESSAGE("${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR}")
  MESSAGE(FATAL_ERROR "Running cmake in the source directory is not supported.\nPlease delete CMakeCache.txt file and run cmake in a separate directory but pointing it to this source directory.")
ENDIF()

project(SGM VERSION 1.0 LANGUAGES NONE)

cmake_policy(SET CMP0048 NEW)

include(${CMAKE_CURRENT_SOURCE_DIR}/SGMFunctions.cmake)
sgm_project_setup()

# build settings for all projects
OPTION(BUILD_SHARED_LIBS "Build with shared libraries" OFF)
OPTION(CMAKE_BUILD_TYPE "Build type" Release)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# path to test data model files for use in C++
# to run the model tests you need to have the sgm-models and sgm-models-ouo
# repo cloned as alongside the sgm repo
set(SGM_MODELS_DIRECTORY "${CMAKE_SOURCE_DIR}/../sgm-models")
add_definitions(-DSGM_MODELS_DIRECTORY=\"${SGM_MODELS_DIRECTORY}\")
set(SGM_MODELS_OUO_DIRECTORY "${CMAKE_SOURCE_DIR}/../sgm-models-ouo")
add_definitions(-DSGM_MODELS_OUO_DIRECTORY=\"${SGM_MODELS_OUO_DIRECTORY}\")

# Multithreaded use
option(BUILD_MULTITHREADED "Build SGM with multithreaded capabilities." OFF)
if(BUILD_MULTITHREADED)
  message(STATUS "Build SGM multithreaded")
  find_package(Threads)
  add_definitions(-DSGM_MULTITHREADED=1)
  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native -funroll-loops")
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native -funroll-loops")
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    # using Intel C++
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # using Visual Studio C++
  endif()
endif()

# Line Coverage
#
# Add variable "cmake -DCOVERAGE=ON" to enable the options to the compile, should work with gcc or clang compiler.
# and execute the test executables once before "make coverage"
if(CMAKE_SYSTEM_NAME MATCHES Linux OR CMAKE_SYSTEM_NAME MATCHES Darwin)
  if (COVERAGE)
    SET(COVERAGE_COMPILE_FLAGS "-g -O0 --coverage")
    SET(COVERAGE_LINK_FLAGS "--coverage")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_COMPILE_FLAGS}" )
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COVERAGE_LINK_FLAGS}" )
  endif (COVERAGE)
  add_custom_target(coverage
          WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
          COMMAND test ! -e coverage || mv coverage coverage_`date -r coverage +%Y-%m-%d_%H-%M-%S`
          COMMAND cd Source && find . -name "*.gcda" -type f -delete # remove old data files
          COMMAND ${PROJECT_BINARY_DIR}/bin/sgm_tests                # run tests
          COMMAND mkdir -p coverage && cd coverage && gcovr --object-directory=${PROJECT_BINARY_DIR} -r ${CMAKE_SOURCE_DIR}/Source --xml -o coverage.xml #`date +%Y-%m-%d-%H-%M-%S`_coverage.xml
          COMMAND mkdir -p coverage && cd coverage && gcovr --object-directory=${PROJECT_BINARY_DIR} -r ${CMAKE_SOURCE_DIR}/Source --html --html-details -o coverage.html
          COMMENT "Build coverage.html summarizing coverage of tests using gcovr tool."
          )
endif()

add_definitions(-D_SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING)

# use the version number from the CMake project in the compile
add_definitions(-DSGM_PRODUCT_VERSION=${SGM_VERSION}) # not a quoted string, like a float literal


# convenience function to set common properties on SGM libraries
function(add_sgm_library NAME)
  # add the library
  add_library(${NAME} ${ARGN})

  # set version information
  #set_target_properties(${NAME} PROPERTIES
  #                              SOVERSION ${SGM_VERSION_MAJOR}.${SGM_VERSION_MINOR}
  #                              VERSION ${SGM_VERSION_MAJOR}.${SGM_VERSION_MINOR}} )

  # dll interface for windows requires definition of SGM_MAIN
  string(TOUPPER ${NAME} NAMEUPPER)
  set_target_properties(${NAME} PROPERTIES DEFINE_SYMBOL  "${NAMEUPPER}_MAIN" )

  # on Windows, 64 bit library gets a different name
  if(CMAKE_SYSTEM_NAME MATCHES Windows AND CMAKE_C_SIZEOF_DATA_PTR MATCHES 8)
    set_target_properties(${NAME} PROPERTIES OUTPUT_NAME ${NAME}x64)
  endif()

  # add D suffix to debug library names
  set_target_properties(${NAME} PROPERTIES DEBUG_POSTFIX "D")

  # installation of library
  install(TARGETS ${NAME} EXPORT SGM
        RUNTIME DESTINATION bin COMPONENT Runtime
        LIBRARY DESTINATION lib COMPONENT Runtime
        ARCHIVE DESTINATION lib COMPONENT Development)

endfunction()

# the projects we are building
add_subdirectory(Source)
add_subdirectory(Apps)
add_subdirectory(Tests)
add_subdirectory(Tests/lib/googletest-release-1.8.0)

option(BUILD_MODEL_VIEWER "Build the Qt based model viewer application." OFF)
if(BUILD_MODEL_VIEWER)
  add_subdirectory(ModelViewer)
endif()

# installation for all projects
install(EXPORT SGM FILE SGMConfig.cmake
        DESTINATION cmake
        COMPONENT Development)

