#include "simulate.hpp"
#include "two_stage.hpp"
#include "utilities.hpp"

#include <iostream>
#include <map>
// #include <set>
#include <fstream>

using namespace lemon;

int main(int argc, char * argv[]) {

    int number_nodes = std::stoi(argv[1]);
    int number_scenarios = std::stoi(argv[2]);;
    // int number_nodes = 10;

    bool on_cluster = true;
    bool save_problems = false;
    bool time = true;               // ob ich den lp-alg timen/tracken will

    // int number_minus_edges = 20;

    // Alg::GreedyApprox, Alg::Schranke4b, Alg::Optimal, Alg::LPApprox

    NRandomScenarioCreator s(number_scenarios, rng);    
    RandomTestCreator n(0., 10., rng);

    // std::vector<double> ps {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    // std::vector<int> runs {500, 500, 500, 100, 100, 100, 100, 100, 100, 100, 100};//, 50, 50, 50, 50};
    std::vector<double> ps {0.00, 0.5};
    std::vector<int> runs {500, 500};

    for (int i=0; i<ps.size(); i++) {
        TreePlusP ensemble2(number_nodes, s, n, rng, ps[i]);
        // std::cout << "Edges:" << ensemble2.two_stage_problem.get_number_edges() << std::endl;
        // FullyConnectedMinusEdges ensemble2(33, s, n, rng, 29);


        ensemble2.initialize();
        std::set<Alg> s {Alg::Optimal};//, Alg::Schranke4b, Alg::GreedyApprox};
        // std::string ordner_name = "TimeTest_" + std::to_string(number_nodes) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
        std::string ordner_name = "Optimum_TimeTest_" + std::to_string(number_nodes) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
        simulate(runs[i], ensemble2, s, ordner_name, on_cluster, save_problems);
    }
}