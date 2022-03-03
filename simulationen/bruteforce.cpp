#include <vector>

#include "../two_stage.hpp"
#include "../utilities.hpp"

int main() {

    // --- zweite Simulation
    // std::vector<double> scenarioProbabilities {0.2, 0.3, 0.5};
    // std::vector<double> firstStageWeights {0.5, 1., 10., 10., 1., 1., 6., 8.5, 0.6, 1.2, 1.1, 15., 2., 0.1, 0.2, 0.5, 1., 10., 10., 1., 1., 6., 8.5, 0.6, 1.2, 1.1, 15., 2., 0.1, 0.2, 0.5, 1., 10., 10., 1., 1., 6., 8.5, 0.6, 1.2, 1.1, 15., 2., 0.1, 0.2};
    // std::vector<std::vector<double>> secondStageWeights {{{2.5, 1.8, 10.5, 1.2, 0.05, 12.2, 6., 8.5, 1.6, 3.2, 0.1, 15., 2.25, 0.6, 0.1}, {19., 90., 5., 18., 20., 12.73, 3.14, 8.5, 9.6, 3.1, 1.1, 3.7, 2., 0.6, 2.2}, {0.5, 1., 3., 10., 1.6, 1., 0.6, 0.5, 14.6, 1.2, 3.2, 1.3, 0.02, 0.1, 5.2}, {2.5, 1.8, 10.5, 1.2, 0.05, 12.2, 6., 8.5, 1.6, 3.2, 0.1, 15., 2.25, 0.6, 0.1}, {19., 90., 5., 18., 20., 12.73, 3.14, 8.5, 9.6, 3.1, 1.1, 3.7, 2., 0.6, 2.2}, {0.5, 1., 3., 10., 1.6, 1., 0.6, 0.5, 14.6, 1.2, 3.2, 1.3, 0.02, 0.1, 5.2}}};

    // ich glaube, die secondSTageWeights sind nicht so geschrieben, wie eigentlich vorgesehen (vorgesehen ist ein vector pro szenario fuer alle Kanten), so wie sie da stehen, sind viele second stage Kosten Null glaube ich

    // --- dritte Simulation: nochmal andere Gewichte, wo die ersten Kanten in der ersten Stage nicht so billig sind
    std::vector<double> scenarioProbabilities {0.4, 0.4, 0.2};
    std::vector<double> firstStageWeights {4.15, 6.05, 2.93, 6.93, 5., 6.17, 8.76, 7.8, 7.18, 0.89, 2.49, 3.52, 4.01, 0.84, 7.41, 5.89, 7.2, 7.88, 1.95, .38, 8.07, 2.88, 1.16, 1.39, 2.62, 1.37, 3.56, 1.67, 4.79, 4., 0.39, 7.43, 5.82, .97, 4.73, 7.9, 5.31, 0.9, 8.9, 8.39, 3.42, 3.63, 5.72, 4.9, 1.57};
    std::vector<std::vector<double>> secondStageWeights {{{4.4, 8.8, 4.9, 6., 7.3, 0.8, 8.2, 8.7, 2.3, 5.5, 1.5, 4.1, 5.8, 6.1, 3.6, 2.4, 7.8, 3.4, 3.4, 3.5, 5., 5.3, 7.9, 7.3, 0.2, 7.1, 3.7, 2.8, 7.4, 8.7, 6.7, 3.6, 2.4, 6.5, 8.1, 1.4, 8.9, 3.6, 4.2, 0.4, 8.5, 6.4, 0.3, 2.1, 3.1}, 
    {8.8, 3.9, 3.2, 8.2, 4.9, 6.8, 6.8, 4.3, 4.1, 6.7, 3.2, 4.6, 5.6, 7.5, 7.5, 6.3, 8.5, 4,3, 2.8, 2.8, 0.5, 4.9, 1.8, 2.9, 5.9, 6.3, 4.5, 8.2, 1.1, 3.8, 2.8, 5.5, 3.9, 3.8, 4.2, 5.7, 3.6, 6.7, 8.1, 5.1, 4.1, 3.6, 3.8, 6.8, 2.9},
    {4.9, 4.7, 5.8, 4.3, 4.5, 1.8, 1.8, 5.7, 3.6, 7.7, 0.6, 2.9, 3.1, 3.7, 3.8, 3.2, 3.0, 6.6, 7.6, 8.5, 0.5, 1.1, 0.3, 5.0, 3.9, 8.2, 7.4, 5.4, 4.7, 3.6, 6.3, 3.7, 1.5, 5.6, 5.7, 5.7, 3.6, 1.9, 4.5, 5.2, 2.8, 5.3, 7.9, 6.5, 1.8}}};

    FullyConnectedTwoStageMST mst(10, firstStageWeights, secondStageWeights, scenarioProbabilities);


    // --- vierte Simulation: 12 Knoten fully connected (entspricht 66 Kanten) mit 5 Szenarien und Kantenggewichten random zwischen 0 und 10

    // int number_nodes = 15;
    // int number_edges = 105;
    // int number_scenarios = 5;

    // auto scenarioProbabilities = calcScenarioProbabilities(number_scenarios, rng);
    // std::uniform_real_distribution<double> dist(0., 10.);                

    // std::vector<double> firstStageWeights;

    // for (int i=0; i<number_edges; i++) {
    //     firstStageWeights.push_back(dist(rng));
    // }

    // std::vector<std::vector<double>> secondStageWeights;
    // for (int i=0; i<number_scenarios; i++) {
    //     std::vector<double> v;
    //     for (int j=0; j<number_edges; j++) {
    //         v.push_back(dist(rng));
    //     }
    //     secondStageWeights.push_back(v);
    // } 

    // FullyConnectedTwoStageMST mst(number_nodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

    // --- fuenfte Simulation 100 Knoten, fully connected (4950 Kanten) mit 5 Szenarien mit Kantengewichten random zwischen 2 und 10, ABER: DIE ERSTEN KANTEN SIND IN STAGE 1 billger als alle anderen

    // int number_nodes = 100;
    // int number_edges = 4950;
    // int number_scenarios = 5;

    // auto scenarioProbabilities = calcScenarioProbabilities(number_scenarios, rng);
    // std::uniform_real_distribution<double> dist(1., 2.);                

    // std::vector<double> firstStageWeights;

    // for (int i=0; i<number_edges; i++) {
    //     firstStageWeights.push_back(dist(rng) + 50);
    // }

    // for (int i=0; i<50; i++) {
    //     firstStageWeights[i] = 0.1;
    // }

    // std::vector<std::vector<double>> secondStageWeights;
    // for (int i=0; i<number_scenarios; i++) {
    //     std::vector<double> v;
    //     for (int j=0; j<number_edges; j++) {
    //         v.push_back(dist(rng));
    //     }
    //     secondStageWeights.push_back(v);
    // } 

    // FullyConnectedTwoStageMST mst(number_nodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

    // auto res = bruteForceEnumeration(mst.g, mst.firstStageWeights, mst.secondStageProbabilities, mst.secondStageWeights);
    auto res = mst.bruteforce_new();
    // mst.save_bruteforce_first_stage_map("bruteforce_nur_zyklus_simulation");

    double expected_costs = mst.calculate_expected_from_bool_map(mst.bruteforce_first_stage_map);
    std::cout << "Mit dieser Auswahl hat man Gesamterwartungskosten von : " << expected_costs << std::endl;

    return 0;
}
