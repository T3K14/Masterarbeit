cmake_minimum_required(VERSION 3.15.0)

set (Sources
    two_stage_test.cpp
)
if(NOT On_Cluster)
add_executable(tests ${Sources})
target_link_libraries(tests PUBLIC
    gtest_main
    two_stage
    gtest
)
endif()

if(On_Cluster)
add_executable(approx_tests approximate_test.cpp)
target_link_libraries(approx_tests PUBLIC
    gtest_main
    two_stage
    gtest
    two_stage_gurobi
    gurobi_c++	
    gurobi91
)
endif()

add_test(
    NAME two_stage_test
    COMMAND tests
)

add_test(
    NAME approximate_test
    COMMAND approx_tests
)
