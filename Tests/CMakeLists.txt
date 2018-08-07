project(sgm_test)

set(CMAKE_CXX_STANDARD 11)

#add_subdirectory(lib/googletest-release-1.8.0) # defined in parent CMakeLists.txt

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

# sgm_tests
add_executable(sgm_tests
        boxtree_check.cpp
        interval_check.cpp
        volume_check.cpp
        intersection_check.cpp
        test_utility.cpp
        math_check.cpp
        models_ouo_check.cpp
        )
target_link_libraries(sgm_tests gtest_main gtest)
target_link_libraries(sgm_tests SGM)

# sgm_model_tests
add_executable(sgm_model_tests
        models_check.cpp)
target_link_libraries(sgm_model_tests gtest_main gtest)
target_link_libraries(sgm_model_tests SGM)

# boxtree_timing
add_executable(boxtree_timing
        boxtree_timing.cpp)
target_link_libraries(boxtree_timing SGM)

# threadpool_tests NOTE: threadpool not compiling on VS2013
#add_executable(threadpool_tests
#        threadpool_check.cpp)
#target_link_libraries(threadpool_tests gtest_main gtest)
