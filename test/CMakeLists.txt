cmake_minimum_required(VERSION 3.20.0)

set (Sources
    two_stage_test.cpp
)

add_executable(tests ${Sources})
target_link_libraries(tests PUBLIC
    gtest_main
    two_stage
    gtest
)

add_test(
    NAME two_stage_test
    COMMAND tests
)