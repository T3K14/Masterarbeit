#ifndef TWO_STAGE_HPP
#define TWO_STAGE_HPP

#include <lemon/list_graph.h>
#include <random>
#include <memory>


struct RNG{
    std::mt19937 eng;
};

class EdgeCostCreator {

    private:
        RNG & rng;
        // std::uniform_real_distribution<double> uniformDist;


    public:
        ~EdgeCostCreator() = default;
        EdgeCostCreator(RNG & randGen);
        // lemon::ListGraph::EdgeMap<double> createUniformCosts(const lemon::ListGraph & g, double a, double b);
        std::unique_ptr<lemon::ListGraph::EdgeMap<double>> createUniformCosts(const lemon::ListGraph & g, double a, double b);


};
#endif // TWO_STAGE_HPP