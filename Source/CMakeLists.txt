project(SGM)

# glob for sources (manual re-run of cmake required if list of sources changes)
file(GLOB SGM_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)
file(GLOB SGM_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} Headers/*.h)
file(GLOB SGM_PUBLIC_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} Public_Headers/*.h)


# create the SGM library with sources and headers
add_sgm_library(SGM ${SGM_SOURCES} ${SGM_HEADERS} ${SGM_PUBLIC_HEADERS})

# the include directories needed to build this code
# PUBLIC includes directories are added to libraries that link against SGM
target_include_directories(SGM PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Public_Headers>
                                      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Headers>
                                      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                      $<INSTALL_INTERFACE:include>)


# installation
install(FILES ${SGM_PUBLIC_HEADERS} 
        DESTINATION include COMPONENT Development)
