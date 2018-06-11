project(sgm_test)

set(CMAKE_CXX_STANDARD 11)

add_subdirectory(lib/googletest-release-1.8.0)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_executable(sgm_tests
        interval_check.cpp
        )

target_link_libraries(sgm_tests gtest gtest_main SGM)

add_executable(boxtree_timing
        boxtree_timing.cpp)

target_link_libraries(boxtree_timing SGM)
