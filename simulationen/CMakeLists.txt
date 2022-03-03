cmake_minimum_required(VERSION 3.15.0)

if(On_Cluster)

add_executable(erste_approx approx.cpp)
target_link_libraries(erste_approx PUBLIC
    two_stage
    simulate
    two_stage_gurobi
)

add_executable(approx_long approx_long.cpp)
target_link_libraries(approx_long PUBLIC
    two_stage
    simulate
    two_stage_gurobi
)

add_executable(approx_time approx_time_test.cpp)
target_link_libraries(approx_time PUBLIC
two_stage
simulate
two_stage_gurobi
)

endif()

add_executable(erste_bruteforce bruteforce.cpp)
target_link_libraries(erste_bruteforce PUBLIC
two_stage
)

