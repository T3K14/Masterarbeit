#include "simulate.hpp"
#include "two_stage.hpp"
#include "utilities.hpp"

#include <iostream>
#include <map>

using namespace lemon;

int main() {

    // will schauen, ob Vererbung klappt

    NRandomScenarioCreator s(3, rng);    
    RandomTestCreator n(0., 10., rng);

    // Tree tree(10, s, n, rng);
    // tree.recreate();

    FullyConnectedMinusEdges ensemble(6, s, n, rng, 5);
    ensemble.initialize();

    // double res = solve_relaxed_lp(mst);

    ensemble.save_current_graph("minus_test");

    int number_edges = 44;
    int number_nodes;
    int number_minus_edges;

    for (int i=2; i<=10; i++) {
        // fc_edges_to_nodes.insert(std::make_pair(i, BinomialCoefficient(i, 2));)
        // std::cout << BinomialCoefficient(i, 2) << "\n";

        if (number_edges <= BinomialCoefficient(i, 2)) {
            number_nodes = i;
            number_minus_edges = BinomialCoefficient(i, 2) - number_edges;
            std::cout << "Nodes: " << number_nodes << ", fc: " << BinomialCoefficient(i, 2) << ", minus_edges: " << number_minus_edges << "\n";
            break;
        }
    }

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