

macro(sgm_project_setup)

  set(CMAKE_LINK_DEPENDS_NO_SHARED ON)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  if(POLICY CMP0061)
    cmake_policy(SET CMP0061 NEW)
  endif()
  if(POLICY CMP0065)
    cmake_policy(SET CMP0065 NEW)
  endif()

  # want position independent code for everything
  # much of the code put in static libraries is later included in shared libraries
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)


  # set the default compiler names to a more common name found in recognized compiler installations
  # some compiler installations do not include "cc" which is CMake's default
  set(CMAKE_CXX_COMPILER_NAMES g++)

  enable_language(CXX)

  # if we found sierra compiler, set rpath to allow running without LD_LIBRARY_PATH
#  if(CMAKE_CXX_COMPILER MATCHES "sierra" AND NOT CMAKE_EXE_LINKER_FLAGS MATCHES "sierra")
#    foreach(dir ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})
#      if(dir MATCHES "sierra")
#        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath,${dir}")
#      endif()
#    endforeach()
#  endif()


  if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(CMAKE_INSTALL_RPATH "\$ORIGIN")
    
    # not all 3rd party libraries are compatible with new-dtags
#    if(NOT CMAKE_SHARED_LINKER_FLAGS MATCHES dtags)
#      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
#      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
#      set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -Wl,--disable-new-dtags")
#    endif()
  endif()


  if(NOT CMAKE_INSTALL_BINARY_DIR)
    SET(CMAKE_INSTALL_BINARY_DIR "lib")
  endif()
  if(NOT CMAKE_INSTALL_INCLUDE_DIR)
    SET(CMAKE_INSTALL_INCLUDE_DIR "include")
  endif()

  # Increase warning level
  if(CMAKE_C_COMPILER_ID MATCHES GNU OR CMAKE_C_COMPILER_ID MATCHES Clang)
    if(NOT CMAKE_C_FLAGS MATCHES Wall)
      set(CMAKE_C_FLAGS "-Wall -WX -Wsign-compare ${CMAKE_C_FLAGS}")
    endif()
    if(NOT CMAKE_CXX_FLAGS MATCHES Wall)
      set(CMAKE_CXX_FLAGS "-Wall -WX -Wsign-compare ${CMAKE_CXX_FLAGS}")
    endif()
  endif()

  # extra compile flags
  if(CMAKE_SYSTEM_NAME MATCHES Linux AND CMAKE_COMPILER_IS_GNUCXX)
    if(NOT CMAKE_SHARED_LINKER_FLAGS MATCHES "no-undefined")
      set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -Wl,--no-undefined")
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
    endif()
#    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "5.0")
#      if(NOT CMAKE_CXX_FLAGS MATCHES _GLIBCXX_USE_CXX11_ABI)
#        set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0 ${CMAKE_CXX_FLAGS}")
#      endif()
#    endif()
  endif()
  
  if(NOT MSVC AND NOT CMAKE_CXX_FLAGS MATCHES fvisibility)
    set(CMAKE_CXX_FLAGS "-fvisibility=hidden ${CMAKE_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "-fvisibility=hidden ${CMAKE_C_FLAGS}")
  endif()

#   default to a debug build if nothing was specified, the first time
#   but still allow someone to go in the cache file and set the build type
#   to nothing if they wanted to
#  IF(UNIX)
#    IF(NOT SET_FIRST_BUILD_TYPE)
#      SET(SET_FIRST_BUILD_TYPE ON CACHE INTERNAL "internal var to help set debug the first time")
#      IF(NOT CMAKE_BUILD_TYPE)
#        SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
#      ENDIF(NOT CMAKE_BUILD_TYPE)
#    ENDIF(NOT SET_FIRST_BUILD_TYPE)
#  ENDIF(UNIX)


  # warn about VLAs which don't compile under MSVC
  if(CMAKE_C_COMPILER_ID MATCHES GNU OR CMAKE_C_COMPILER_ID MATCHES Clang)
    if(NOT CMAKE_C_FLAGS MATCHES Wvla)
      set(CMAKE_C_FLAGS "-Wvla ${CMAKE_C_FLAGS}")
    endif()
    if(NOT CMAKE_CXX_FLAGS MATCHES Wvla)
      set(CMAKE_CXX_FLAGS "-Wvla ${CMAKE_CXX_FLAGS}")
    endif()
  endif()

  if(MSVC)
    # Force to always compile with W4
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
      string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
      set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    endif()
    # Force to always compile with WX
    if(CMAKE_CXX_FLAGS MATCHES "/WX[-]")
      string(REGEX REPLACE "/WX[-]" "/WX" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
    endif()
  endif(MSVC)

  include(GenerateExportHeader)

endmacro()
