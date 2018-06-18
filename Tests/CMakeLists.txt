project(sgm_test)

set(CMAKE_CXX_STANDARD 11)

add_subdirectory(lib/googletest-release-1.8.0)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_executable(sgm_tests
        boxtree_check.cpp
        interval_check.cpp
        volume_check.cpp
        )

target_link_libraries(sgm_tests gtest_main gtest)
target_link_libraries(sgm_tests SGM)

add_executable(boxtree_timing
        boxtree_timing.cpp)

target_link_libraries(boxtree_timing SGM)
