cmake_minimum_required(VERSION 3.1)

IF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  MESSAGE("${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR}")
  MESSAGE(FATAL_ERROR "Running cmake in the source directory is not supported.\nPlease delete CMakeCache.txt file and run cmake in a separate directory but pointing it to this source directory.")
ENDIF()


project(SGMall NONE)

include(${CMAKE_CURRENT_SOURCE_DIR}/SGMFunctions.cmake)
sgm_project_setup()


# hard coded version number

#set(SGM_VERSION_MAJOR 1)
#set(SGM_VERSION_MINOR 0)


# build settings for all projects
set(BUILD_SHARED_LIBS ON)

#if(CMAKE_SYSTEM_NAME MATCHES Darwin)
#  add_definitions(-DSGM_OSX)
#endif()

#if(CMAKE_SYSTEM_NAME MATCHES Windows)
#  add_definitions(-D_USE_MATH_DEFINES)
#endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


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
        LIBRARY DESTINATION bin COMPONENT Runtime
        ARCHIVE DESTINATION bin COMPONENT Development)

endfunction()

# the projects we are building
add_subdirectory(Source)
add_subdirectory(Apps)



# installation for all projects
install(EXPORT SGM FILE SGMConfig.cmake
        DESTINATION cmake
        COMPONENT Development)

