project(sgm_test)

set(CMAKE_CXX_STANDARD 11)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

set(SGM_GTEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/boxtree_check.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Boolean_check.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/create_check.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/intersection_check.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/interval_check.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/math_check.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/models_single_check.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/volume_check.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_utility.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/area_check.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/transform_check.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/delete_check.cpp
    )

# add source variable in parent scope to be able to build viewer with gtests
set(SGM_VIEWER_GTEST_SOURCES ${SGM_GTEST_SOURCES} PARENT_SCOPE)

#
find_package(Threads)

add_executable(sgm_tests ${SGM_GTEST_SOURCES})
target_link_libraries(sgm_tests gtest_main gtest SGM)

add_executable(sgm_model_tests
               models_check.cpp
               test_utility.cpp)
target_link_libraries(sgm_model_tests gtest_main gtest SGM)

add_executable(boxtree_timing
               boxtree_timing.cpp)
target_link_libraries(boxtree_timing SGM)

#add_executable(threadpool_tests
#        threadpool_check.cpp)
#target_link_libraries(threadpool_tests gtest_main gtest)
