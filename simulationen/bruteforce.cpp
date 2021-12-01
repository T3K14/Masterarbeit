#include <vector>

#include "../two_stage.hpp"
#include "../utilities.hpp"

int main() {

    // Problemdefinition
    // auskommentiert: 6 Knoten
    std::vector<double> scenarioProbabilities {0.2, 0.3, 0.5};
    // std::vector<double> firstStageWeights {0.5, 1., 10., 10., 1., 1., 6., 8.5, 0.6, 1.2, 1.1, 15., 2., 0.1, 0.2};
    // std::vector<std::vector<double>> secondStageWeights {{{2.5, 1.8, 10.5, 1.2, 0.05, 12.2, 6., 8.5, 1.6, 3.2, 0.1, 15., 2.25, 0.6, 0.1}, {19., 90., 5., 18., 20., 12.73, 3.14, 8.5, 9.6, 3.1, 1.1, 3.7, 2., 0.6, 2.2}, {0.5, 1., 3., 10., 1.6, 1., 0.6, 0.5, 14.6, 1.2, 3.2, 1.3, 0.02, 0.1, 5.2}}};
    std::vector<double> firstStageWeights {0.5, 1., 10., 10., 1., 1., 6., 8.5, 0.6, 1.2, 1.1, 15., 2., 0.1, 0.2, 0.5, 1., 10., 10., 1., 1., 6., 8.5, 0.6, 1.2, 1.1, 15., 2., 0.1, 0.2, 0.5, 1., 10., 10., 1., 1., 6., 8.5, 0.6, 1.2, 1.1, 15., 2., 0.1, 0.2};
    std::vector<std::vector<double>> secondStageWeights {{{2.5, 1.8, 10.5, 1.2, 0.05, 12.2, 6., 8.5, 1.6, 3.2, 0.1, 15., 2.25, 0.6, 0.1}, {19., 90., 5., 18., 20., 12.73, 3.14, 8.5, 9.6, 3.1, 1.1, 3.7, 2., 0.6, 2.2}, {0.5, 1., 3., 10., 1.6, 1., 0.6, 0.5, 14.6, 1.2, 3.2, 1.3, 0.02, 0.1, 5.2}, {2.5, 1.8, 10.5, 1.2, 0.05, 12.2, 6., 8.5, 1.6, 3.2, 0.1, 15., 2.25, 0.6, 0.1}, {19., 90., 5., 18., 20., 12.73, 3.14, 8.5, 9.6, 3.1, 1.1, 3.7, 2., 0.6, 2.2}, {0.5, 1., 3., 10., 1.6, 1., 0.6, 0.5, 14.6, 1.2, 3.2, 1.3, 0.02, 0.1, 5.2}}};



    FullyConnectedTwoStageMST mst(10, firstStageWeights, secondStageWeights, scenarioProbabilities);

    // auto res = bruteForceEnumeration(mst.g, mst.firstStageWeights, mst.secondStageProbabilities, mst.secondStageWeights);
    auto res = mst.bruteforce_new();
    mst.save_bruteforce_first_stage_map("bruteforce_zweite_simulation_new");

    double expected_costs = mst.calculate_expected_from_bool_map(mst.bruteforce_first_stage_map);
    std::cout << "Mit dieser Auswahl hat man Gesamterwartungskosten von : " << expected_costs << std::endl;

    return 0;
}