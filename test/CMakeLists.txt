cmake_minimum_required(VERSION 3.15.0)

set (Sources
    two_stage_test.cpp
)

add_executable(tests ${Sources})
target_link_libraries(tests PUBLIC
    gtest_main
    two_stage
    gtest
)


add_executable(approx_tests approximate_test.cpp)
target_link_libraries(approx_tests PUBLIC
    gtest_main
    two_stage
    gtest
)

add_test(
    NAME two_stage_test
    COMMAND tests
)

add_test(
    NAME approximate_test
    COMMAND approx_tests
)