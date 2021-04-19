#include <chrono>
// #include <memory>
#include "two_stage.hpp"
#include <lemon/lgf_writer.h>
#include <lemon/kruskal.h>

// EdgeCostCreator::EdgeCostCreator(RNG & randGen) : rng(randGen) {}

std::unique_ptr<lemon::ListGraph::EdgeMap<double>> EdgeCostCreator::createUniformCosts(const lemon::ListGraph & g, double a, double b, std::mt19937 & rng) {
// auto 

    /* rng is already seeded and should only be seeded once
    get seed
    unsigned int seed1 = std::chrono::system_clock::now().time_since_epoch().count();
    rng.eng.seed(seed1);        // is no seed sequence
    */

    std::uniform_real_distribution<double> dist(a, b);

    // lemon::ListGraph::EdgeMap<double> costMap(g);

    // unique_ptr to a ListGraph::EdgeMap<double> 
    auto costMapPtr = std::make_unique<lemon::ListGraph::EdgeMap<double>>(g);

    for (lemon::ListGraph::EdgeIt e(g); e !=lemon::INVALID; ++e) {
        (*costMapPtr)[e] = dist(rng);
    }
    return costMapPtr;      //wird gemoved
}

template<typename T>
T twoStageSetting(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<T> & firstStageCosts, const lemon::ListGraph::EdgeMap<T> & secondStageCosts, bool save) {

    //TEST: TwoStageSuite/Test1

    EdgeCostCreator ecc;

    // range values for the real uniform distribution
    double a = 1.0;
    double b = 99.0;

    // create costs for the first stage
    // auto stage1Ptr = ecc.createUniformCosts(g, a, b, rng);

    // create costs for the second stage
    // auto stage2Ptr = ecc.createUniformCosts(g, a, b, rng);


    // create edgeMap that stores the minimum value of both stages for each edge
    lemon::ListGraph::EdgeMap<T> minMap(g); 
    // wie iteriere ich ueber die edges??

    for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
        minMap[e] = (firstStageCosts[e] < secondStageCosts[e] ? firstStageCosts[e] : secondStageCosts[e]);
    }

    // do the MST calculation
    lemon::ListGraph::EdgeMap<bool> kruskalResMap(g);
    double totalCosts = lemon::kruskal(g, minMap, kruskalResMap);


    if (save) {
        std::string outputPath(R"(D:\uni\Masterarbeit\Code\output\output.lgf)");
        lemon::GraphWriter(g, outputPath).edgeMap("first_stage", firstStageCosts).edgeMap("second_stage", secondStageCosts).edgeMap("final_selection", minMap).edgeMap("MST_selection", kruskalResMap).run();

    }
    return totalCosts;
}


template<typename T>        // T is the edge cost type
void bruteForceEnumeration(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<T> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::reference_wrapper<lemon::ListGraph::EdgeMap<T>>> & scenarioSecondStageCosts) {
    // std::cout << "hi\n";

    

}


template void bruteForceEnumeration<double> (const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::reference_wrapper<lemon::ListGraph::EdgeMap<double>>> & scenarioSecondStageCosts);
template int twoStageSetting<int>(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<int> & firstStageCosts, const lemon::ListGraph::EdgeMap<int> & secondStageCosts, bool save);