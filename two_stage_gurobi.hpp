#pragma once

#include <lemon/list_graph.h>
#include "utilities.hpp"
#include "gurobi_c++.hpp"
#include "two_stage.hpp"
#include <vector>

void solve_relaxed_lp(lemon::ListGraph::EdgeMap<std::vector<double>> & result_optimized_values_map);
void approximate(lemon::ListGraph::EdgeMap<std::vector<double>> & result_optimized_values_map)
