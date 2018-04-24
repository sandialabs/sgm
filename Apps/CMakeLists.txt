# glob for sources (manual re-run of cmake required if list of sources changes)
file(GLOB SGMApps_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

# create the SGM library with sources and headers
add_executable(RunTests ${SGMApps_SOURCES})

target_include_directories(RunTests PRIVATE ../Source/Public_Headers)

target_link_libraries(RunTests PRIVATE SGM)

ADD_DEFINITIONS( -DSGM_TEST_INPUT_DIR=\"${CMAKE_SOURCE_DIR}/Tests\" )
ADD_DEFINITIONS( -DSGM_TEST_OUTPUT_DIR=\"${CMAKE_SOURCE_DIR}\" )

# the include directories needed to build this code
# PUBLIC includes directories are added to libraries that link against SGM
target_include_directories(RunTests PRIVATE $SGM_SOURCE_DIR}/Public_Headers>)

