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

add_test(
    NAME two_stage_test
    COMMAND tests
)

endif()

add_executable(tests2 two_stage_test_new.cpp)
target_link_libraries(tests2 PUBLIC
    gtest_main
    two_stage
    gtest
)

if(On_Cluster)
add_executable(approx_tests approximate_test.cpp)
target_link_libraries(approx_tests PUBLIC
    gtest_main
    two_stage
    simulate
    gtest
    two_stage_gurobi
    gurobi_c++	
    gurobi91
)

add_test(
    NAME approximate_test
    COMMAND approx_tests
)

endif()



add_test(
    NAME two_stage_test_neu
    COMMAND tests2
)

# habe den rausgenommen, weil er nichts macht und weil ich hier auf dem Cluster noch two_stage_gurobi verlinken muesste
#add_executable(simulate_t simulate_test.cpp)
#target_link_libraries(simulate_t PUBLIC
#    gtest_main
#    two_stage
#    gtest
#    simulate
#)

add_test(
    NAME simulate_test
    COMMAND simulate_t
)