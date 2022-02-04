#include "simulate.hpp"
#include "two_stage.hpp"
#include "utilities.hpp"

#include <iostream>
#include <map>

using namespace lemon;

int main() {

    // IRGENDWAS KLAPPT BEI DER KONSTRUKTION NICHT, DIE EDGES SIND LEER ODER SO


    // will schauen, ob Vererbung klappt

    NRandomScenarioCreator s(3, rng);    
    RandomTestCreator n(0., 10., rng);

    // Tree tree(10, s, n, rng);
    // tree.recreate();

    FullyConnected ensemble(4, s, n);
    ensemble.initialize();

    ensemble.do4b();

    std::cout << 1.0000001 << std::endl;
    std::cout << 1.00001 << std::endl;
    std::cout << 1. << std::endl;
    std::cout << 1 << std::endl;

    std::cout << int(0.9999) << std::endl;
    std::cout << int(0.99999) << std::endl;


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