#include <chrono>
// #include <memory>
#include "two_stage.hpp"

EdgeCostCreator::EdgeCostCreator(RNG & randGen) : rng(randGen) {}

std::unique_ptr<lemon::ListGraph::EdgeMap<double>> EdgeCostCreator::createUniformCosts(const lemon::ListGraph & g, double a, double b) {

    // get seed
    unsigned int seed1 = std::chrono::system_clock::now().time_since_epoch().count();
    rng.eng.seed(seed1);        // is no seed sequence

    std::uniform_real_distribution<double> dist(a, b);

    // lemon::ListGraph::EdgeMap<double> costMap(g);

    auto costMapPtr = std::make_unique<lemon::ListGraph::EdgeMap<double>>(g);

    for (lemon::ListGraph::EdgeIt e(g); e !=lemon::INVALID; ++e) {
        (*costMapPtr)[e] = dist(rng.eng);
    }
    return costMapPtr;
}
