# glob for sources (manual re-run of cmake required if list of sources changes)
file(GLOB SGM_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)
file(GLOB SGM_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} Headers/*.h)
file(GLOB SGM_PUBLIC_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} Public_Headers/*.h)
file(GLOB SGM_PUBLIC_HEADERS_INLINE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} Public_Headers/Inline/*.inl)

# create the SGM library with sources and headers
add_sgm_library(SGM ${SGM_SOURCES} ${SGM_HEADERS} ${SGM_PUBLIC_HEADERS})

source_group("Public Headers" FILES ${SGM_PUBLIC_HEADERS})

# the include directories needed to build this code
# PUBLIC includes directories are added to libraries that link against SGM
target_include_directories(SGM PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Public_Headers>
                                      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Headers>
                                      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> 
                                      $<INSTALL_INTERFACE:include>)

generate_export_header(SGM)

# installation
install(FILES ${SGM_PUBLIC_HEADERS} ${CMAKE_CURRENT_BINARY_DIR}/sgm_export.h
        DESTINATION include COMPONENT Development)
install(FILES ${SGM_PUBLIC_HEADERS_INLINE}
        DESTINATION include/Inline COMPONENT Development)
        