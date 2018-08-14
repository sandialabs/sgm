set (RUN_TEST_SOURCE MainRunTests.cpp)
set (1516_APP_SOURCE 1516_App.cpp)

# create the SGM library with sources and headers
add_executable(RunTests ${RUN_TEST_SOURCE})
target_include_directories(RunTests PRIVATE ../Source/Public_Headers)
target_link_libraries(RunTests PRIVATE SGM)

add_executable(SGM_Cover ${1516_APP_SOURCE})
target_include_directories(SGM_Cover PRIVATE ../Source/Public_Headers)
target_link_libraries(SGM_Cover PRIVATE SGM)

ADD_DEFINITIONS( -DSGM_TEST_INPUT_DIR=\"${CMAKE_SOURCE_DIR}/Tests\" )
ADD_DEFINITIONS( -DSGM_TEST_OUTPUT_DIR=\"${CMAKE_SOURCE_DIR}\" )

# the include directories needed to build this code
# PUBLIC includes directories are added to libraries that link against SGM
# target_include_directories(RunTests PRIVATE $SGM_SOURCE_DIR}/Public_Headers>)

