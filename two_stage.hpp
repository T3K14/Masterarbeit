#pragma once

#include <lemon/list_graph.h>
#include <random>
#include <memory>


// struct RNG{
    // std::mt19937 eng;
// };

extern std::mt19937 rng;

class EdgeCostCreator {

    private:
        // RNG & rng;
        // std::uniform_real_distribution<double> uniformDist;


    public:
        ~EdgeCostCreator() = default;
        // EdgeCostCreator(RNG & randGen);
        // lemon::ListGraph::EdgeMap<double> createUniformCosts(const lemon::ListGraph & g, double a, double b);
        std::unique_ptr<lemon::ListGraph::EdgeMap<double>> createUniformCosts(const lemon::ListGraph & g, double a, double b, std::mt19937 & rng);


        // FALLS ICH MAL PARALLELISIEREN MUSS UND DABEI ANDEREN RNG ALS MT19937 NUTZEN WILL/MUSS
        // template<typename RNG>
        // void doRandomStuff(float x, RNG && rng);            // perfect forwarding, universial reference (Scott Meyers)

};

// returns the total weight of the MST
template<typename T>        // T is the edge cost type
T twoStageSetting(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<T> & firstStageCosts, const lemon::ListGraph::EdgeMap<T> & secondStageCosts, bool save=false);
