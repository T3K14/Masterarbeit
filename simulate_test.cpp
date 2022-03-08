#include "simulate.hpp"
#include "two_stage.hpp"
#include "utilities.hpp"
// #include "two_stage_gurobi.hpp"


#include <iostream>
#include <map>
// #include <set>
#include <fstream>

// #include "boost/filesystem.hpp"

using namespace lemon;

int main() {

    int number_scenarios = 15;
    int number_nodes = 15;

    bool on_cluster = true;
    bool save_problems = true;

    int number_minus_edges = 20;

    // Alg::GreedyApprox, Alg::Schranke4b, Alg::Optimal, Alg::LPApprox

    NRandomScenarioCreator s(number_scenarios, rng);    
    RandomTestCreator n(0., 10., rng);

    std::vector<double> ps {0.05};//, 0.1, 0.15, 0.2, 0.25};
    std::vector<int> runs {50};//, 50, 50, 50, 50};
    // std::vector<double> ps;

    // for (int i=0; i<=10; i++) {
    //     double p = 0.1 * i;
    //     ps.push_back(p);
    // }

    // // TreePlusP ensemble2(13, s, n, rng, 0.0);
    // // ensemble2.initialize();
    // // simulate(250, ensemble2, std::set<Alg> {Alg::GreedyApprox, Alg::Schranke4b, Alg::Optimal}, "TreePlusP_13");

    for (int i=0; i<ps.size(); i++) {
        // TreePlusP ensemble2(number_nodes, s, n, rng, ps[i]);
        FullyConnectedMinusEdges ensemble2(number_nodes, s, n, rng, number_minus_edges);


        ensemble2.initialize();
        std::set<Alg> s {Alg::LPApprox};
        simulate(runs[i], ensemble2, s, "First_HPC_Test", on_cluster, save_problems);
    }

    // ensemble2.save_current_graph("treeplus0.5");

    // ensemble.do4b();

    // std::cout << 1.0000001 << std::endl;
    // std::cout << 1.00001 << std::endl;
    // std::cout << 1. << std::endl;
    // std::cout << 1 << std::endl;

    // std::cout << int(0.9999) << std::endl;
    // std::cout << int(0.99999) << std::endl;


    // // teste RandomTestCreator::create_costs()
    // tree.recreate(rng);

    // int counter = 0;
    // for (ListGraph::EdgeIt e(tree.two_stage_problem.g); e!=INVALID; ++e) {
    //     tree.two_stage_problem.firstStageWeights[e] = counter++;
    //     std::cout << tree.two_stage_problem.firstStageWeights[e] << "\n";

    // }

    // std::cout << "\nErase all edges\n";

    // auto edg = tree.two_stage_problem.g.edgeFromId(1);

    // std::cout << tree.two_stage_problem.firstStageWeights[edg] << "\n";

    // tree.erase_all_edges();
    // for (ListGraph::EdgeIt e(tree.two_stage_problem.g); e!=INVALID; ++e) {
    //     std::cout << tree.two_stage_problem.firstStageWeights[e] << "\n";

    // }

    // std::cout << tree.two_stage_problem.firstStageWeights[edg] << "\n";


    // std::cout << "\nEmpty graph\n";
    // ListGraph g;
    // ListGraph::EdgeMap<double> map(g);


    // for (ListGraph::EdgeIt e(g); e!=INVALID; ++e) {
    //     std::cout << map[e] << "\n";

    // }

    // std::cout << "\nrecreate testen\n";
    // tree.recreate(rng);
    // for (ListGraph::EdgeIt e(tree.two_stage_problem.g); e!=INVALID; ++e) {
    //     std::cout << tree.two_stage_problem.firstStageWeights[e] << "\n";

    // }


}