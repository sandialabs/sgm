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

# Find some other way to accomplish it, this breaks Debug build and debugging in IDE's.
#IF(UNIX)
#  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
#ENDIF(UNIX)

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
  add_definitions(-DSGM_MULTITHREADED=1)
  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native -funroll-loops")
      SET(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
      SET(OpenMP_CXX_LIB_NAMES "omp")
      SET(OpenMP_omp_LIBRARY "/usr/local/opt/llvm/lib/libiomp5.dylib")
    endif()
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native -funroll-loops")
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    # using Intel C++
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # using Visual Studio C++
  endif()
  find_package(Threads)
  find_package(OpenMP)
endif()

# increase warnings and disable warnings that do not work
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-user-defined-warnings)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -DDEBUG=2")
endif()
add_definitions(-D_SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING)

## force use of stdc++ library on clang to reveal types in debugger
#if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(--gcc-toolchain=/usr/local/Cellar/gcc@7)
#  add_compile_options(-stdlib=stdlibc++)
#endif()

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

option(BUILD_SGM_TESTS "Build the SGM test executable." OFF)
if(BUILD_SGM_TESTS)
  add_subdirectory(Tests)
  add_subdirectory(Tests/lib/googletest-release-1.8.0)
endif()

option(BUILD_MODEL_VIEWER "Build the Qt based model viewer application." OFF)
if(BUILD_MODEL_VIEWER)
  add_subdirectory(ModelViewer)
endif()

# installation for all projects
install(EXPORT SGM FILE SGMConfig.cmake
        DESTINATION cmake
        COMPONENT Development)

SET(CPACK_PACKAGE_VENDOR "SANDIA")
SET(CPACK_PACKAGE_VERSION_MAJOR "${SGM_VERSION_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${SGM_VERSION_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${SGM_VERSION_PATCH}")
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_LONG "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
set(CPACK_PACKAGE_EXECUTABLES sgm_viewer SGM)

# which components to install
SET(CPACK_COMPONENT_RUNTIME_REQUIRED ON)
SET(CPACK_COMPONENTS_ALL Runtime)
SET(CPACK_COMPONENT_RUNTIME_DISPLAY_NAME "SGM Viewer")

#SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/readme.txt")
#SET(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/readme.txt")
#SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/installer_license.txt")

include(CPack)
