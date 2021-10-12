#pragma once

#include <lemon/list_graph.h>
#include <random>
#include <memory>
// #include <lemon/maps.h>
#include <functional> // for reference_wrapper


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

// calculates the MST and returns the total weight of the MST
template<typename T>        // T is the edge cost type
T twoStageSetting(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<T> & firstStageCosts, const lemon::ListGraph::EdgeMap<T> & secondStageCosts, bool save=false);

// calculates the edgeset that minimizes the expected total costs of the MST for a given second stage edgeset-distribution, by trying out every possible combination
// template<typename T>        // T is the edge cost type
// void bruteForceEnumeration(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<T> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::reference_wrapper<lemon::ListGraph::EdgeMap<T>>> & scenarioSecondStageCosts);
double bruteForceEnumeration(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> & scenarioSecondStageCosts);

// works with EdgeMap<vector<double>> instead of vector<unique_ptr<EdgeMap>>
double bruteForceEnumeration(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const lemon::ListGraph::EdgeMap<std::vector<double>> & scenarioSecondStageCostsEM);

double check(const lemon::ListGraph & g, const std::vector<int> & c, const std::vector<lemon::ListGraph::Edge> & edges, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> & scenarioSecondStageCosts, lemon::ListGraph::EdgeMap<bool> & output, double & currentBest, size_t & optCounter, lemon::ListGraph::EdgeMap<bool> & currentFirstStageSelection);

// works with EdgeMap<vector<double>> instead of vector<unique_ptr<EdgeMap>>
double check(const lemon::ListGraph & g, const std::vector<int> & c, const std::vector<lemon::ListGraph::Edge> & edges, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<lemon::ListGraph::EdgeMap<double>> & scenarioSecondStageCosts, lemon::ListGraph::EdgeMap<bool> & output, double & currentBest, size_t & optCounter, lemon::ListGraph::EdgeMap<bool> & currentFirstStageSelection);

double fourb(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const lemon::ListGraph::EdgeMap<std::vector<double>> & scenarioSecondStageCostsEM);



class SecondStageMap {

    const lemon::ListGraph::EdgeMap<double> & secondStageCosts;
    const std::vector<lemon::ListGraph::Edge> & edges;
    const std::vector<int> & c;

public:

    typedef double Value;
    typedef lemon::ListGraph::Edge Key;

    Value operator[](Key e) const;
  
    SecondStageMap(const lemon::ListGraph::EdgeMap<double> & s, const std::vector<lemon::ListGraph::Edge> & e, const std::vector<int> & _c);

}; 

// const std::vector<lemon::ListGraph::EdgeMap<T>> & scenarioSecondStageCosts
// const std::vector<std::reference_wrapper<lemon::ListGraph::EdgeMap<T>>> & scenarioSecondStageCosts

// von der Klasse koennen spaeter subclasses erben, bei denen klar ist, wie der Graph und die Maps dazu intern erstellt werden sollen
class TwoStageProblem {

    lemon::ListGraph g;

    // kann entweder die zahl an generierten scenarios sein oder die zahl an versuchen, die man beim samplen haben moechte, auch wenn dabei potentiell weniger scenarios
    // am Ende rauskommen
    unsigned int numberScenarios;
    
    lemon::ListGraph::EdgeMap<double> firstStageCosts;

    lemon::ListGraph::EdgeMap<double> secondStageProbabilities;
    lemon::ListGraph::EdgeMap<std::vector<double>> secondStageCosts;

public:

    TwoStageProblem() = delete;    
    virtual ~TwoStageProblem();

    // formuliert das relaxed LP Problem, loesst es und speichert die Ergebnisse in 'result_optimized_values_map'
    friend void solve_relaxed_lp(lemon::ListGraph::EdgeMap<std::vector<double>> & result_optimized_values_map);

    // nimmt die gurobi-lp loesung und ermittelt daraus die approximierte Loesung, die in der EdgeMap final_sirst_stage_map die Vorschlaege fuer in der ersten Stage zu kaufende Kanten 
    // speichert
    void approximate(lemon::ListGraph::EdgeMap<std::vector<double>> & result_optimized_values_map, lemon::ListGraph::EdgeMap<bool> & final_first_stage_map, std::mt19937 & rng);
};

class FullyConnectedTwoStageMST : public TwoStageProblem {

    unsigned int numberNodes;

public:
    
    FullyConnectedTwoStageMST(unsigned int number_scenarios, unsigned int number_nodes);

};

