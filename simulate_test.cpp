#include "simulate.hpp"
#include "two_stage.hpp"
#include "utilities.hpp"

#include <iostream>
#include <map>
// #include <set>
#include <fstream>

using namespace lemon;

int main(int argc, char * argv[]) {

    // int number_nodes = std::stoi(argv[1]);
    // int number_scenarios = std::stoi(argv[2]);;
    int number_scenarios = 5;

    bool on_cluster = true;
    bool save_problems = false;
    bool tracking = true;               // ob ich die Algs timen/tracken will (bisher moeglich fuer lp-approx und die beiden optimalen Algs)

    // int number_minus_edges = 20;

    // Alg::GreedyApprox, Alg::Schranke4b, Alg::Optimal, Alg::LPApprox

    NRandomScenarioCreator sc(number_scenarios, rng);    
    RandomTestCreator rtc(0., 10., rng);
    // GVBilligFirstMittelCreator bfmc(0., 10., rng);

    // std::vector<double> ps {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    // std::vector<int> runs {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};//, 50, 50, 50, 50};
    std::vector<double> ps {0.0};
    std::vector<int> runs {100};

    std::vector<int> nodes {5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

    // for (auto n: nodes) {
    //     TreePlusP ensemble2(n, sc, rtc, rng, ps[0]);
    //     ensemble2.initialize();

    //     std::set<Alg> alg_set {Alg::LPApprox};
    //     // std::string ordner_name = "TimeTest_" + std::to_string(number_nodes) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
    //     std::string ordner_name = "TreeTest_LP3_" + std::to_string(n) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
    //     simulate(runs[0], ensemble2, alg_set, ordner_name, on_cluster, save_problems, tracking);
    // }
    // for (int i=0; i<ps.size(); i++) {
    //     TreePlusP ensemble2(number_nodes, sc, bfc, rng, ps[i]);
    //     // std::cout << "Edges:" << ensemble2.two_stage_problem.get_number_edges() << std::endl;
    //     // FullyConnected ensemble2(11, sc, rtc);

    //     ensemble2.initialize();
    //     std::set<Alg> s {Alg::Optimal2, Alg::LPApprox};
    //     // std::string ordner_name = "TimeTest_" + std::to_string(number_nodes) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
    //     std::string ordner_name = "TreeTest_" + std::to_string(number_nodes) + "_nodes_" + std::to_string(number_scenarios) + "_scenarios";
    //     simulate(runs[i], ensemble2, s, ordner_name, on_cluster, save_problems, tracking);
    // }
}