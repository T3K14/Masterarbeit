#include "simulate.hpp"
#include "two_stage.hpp"
#include "utilities.hpp"

#include <iostream>
#include <map>
// #include <set>
#include <fstream>

using namespace lemon;

int main() {

    int number_scenarios = 10;
    int number_nodes = 500;

    bool on_cluster = true;
    bool save_problems = false;
    bool time = false;

    // int number_minus_edges = 20;

    // Alg::GreedyApprox, Alg::Schranke4b, Alg::Optimal, Alg::LPApprox

    NRandomScenarioCreator s(number_scenarios, rng);    
    RandomTestCreator n(0., 10., rng);

    // std::vector<double> ps {0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    // std::vector<int> runs {500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500};//, 50, 50, 50, 50};
    std::vector<double> ps {0.00};
    std::vector<int> runs {500};

    for (int i=0; i<ps.size(); i++) {
        // TreePlusP ensemble2(number_nodes, s, n, rng, ps[i]);
        // std::cout << "Edges:" << ensemble2.two_stage_problem.get_number_edges() << std::endl;
        FullyConnectedMinusEdges ensemble2(33, s, n, rng, 29);


        ensemble2.initialize();
        std::set<Alg> s {Alg::LPApprox};//, Alg::Schranke4b, Alg::GreedyApprox};
        simulate(runs[i], ensemble2, s, "Test_Edges_499_10_scen_HPC", on_cluster, save_problems);
    }
}