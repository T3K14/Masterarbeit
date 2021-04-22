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


// template<typename T>        // T is the edge cost type
void bruteForceEnumeration(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<T> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::reference_wrapper<lemon::ListGraph::EdgeMap<T>>> & scenarioSecondStageCosts) {
    // std::cout << "hi\n";

    int EdgeCount = lemon::countEdges(g);

    // current best expectation value
    T currentMinEV = 0;

    // brauche ich nicht: ABER ich muss irgendwie die edges fuer den twiddle algorithmus durchnumerieren
    // lemon::ListGraph::EdgeMap<bool> currentBestMST(g);

    // default value is false, so currently no 
    lemon::ListGraph::EdgeMap<bool> currentFirstStageSelection(g, false);

    //solution now: create array of edges and use the index of an edge in this array for the twiddle algorithm
    // std::array<lemon::ListGraph::Edge, EdgeCount> edges; 
    std::vector<lemon::ListGraph::Edge> edges(EdgeCount); 

    size_t c = 0;
    for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
        edges[c++] = e;
    }

    // used as temporary output map of the kruskal algorithm
    lemon::ListGraph::EdgeMap<bool> output(g);      

    // counts how many solutions exist that have the same optimum solution
    size_t counter = 1;     

    // first case: no edge is selected in stage 1
    // loop over all scenarios

    for (int i=0; i < scenarioProbabilities.size(); i++) {
        auto & b = scenarioSecondStageCosts[i];
        T weightedResult = scenarioProbabilities[i] * lemon::kruskal(g, b, output);
        currentMinEV += weightedResult;
    }


    // second case: there are edges selected in the first stage

    T firstStageSum = 0;

    int x,y,z;

    // for every possible number of selectable edges (1,2,3,4,...,M)
    for (int i=1)

        std::vector<int> p(EdgeCount + 2);
        std::vector<int> c(i);       //store the indices of the selected edges for this iteration

        for (int tmp=0; tmp<i; tmp++) {
            c[tmp] = EdgeCount - i + tmp;
        }

        // I want to select i of all [EdgeCount] edges 
        inittwiddle(i, EdgeCount, p);


        // now check the initial case

        T res = check<T>(c, scenarioProbabilities, scenarioSecondStageCosts, p, output)


        while(!twiddle(x, y, z, p)) {

        }

}
template <typename T>
T check(const std::vector<int> & c, const std::vector<lemon::ListGraph::Edge> & edges, const lemon::ListGraph::EdgeMap<T> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::reference_wrapper<lemon::ListGraph::EdgeMap<T>>> & scenarioSecondStageCosts, lemon::ListGraph::EdgeMap<bool> & output) {
    // ermittle Kanten, die in stage 1 gekauft werden (ueber c gegeben)
    
    // teilsumme stage 1 berechnen

    T sumEV = 0;
    for (int i : c) {
        sumEV += firstStageCosts[edges[i]];
    }


    // loop ueber alle scenarien
    for(int i=0; i<scenarioProbabilities.size(); i++) {
        //setze die kosten von diesen Kanten in der map fuer phase 2 auf 0
        SecondStageMap<T> secondStageMap(scenarioSecondStageCosts[i], edges, c);

        // optimalen mst ausrechnen
        T mst = lemon::kruskal(g, secondStageMap, output)

        // Ergebnis mit Wahrscheinlichkeit gewichten und auf Teilsumme stage 2 addieren
        sum += scenarioProbabilities[i] * mst;
    }

        
    return sum;
    // Ergebnis mit bisherigem Optimum speichern, gegebenefalls counter erhoehen und neues bessere Ergebnis speichern, passiert außerhalb der funktion
}   

template <typename T>
class SecondStageMap {

    const lemon::ListGraph::EdgeMap<T> & secondStageCosts;
    const std::vector<lemon::ListGraph::Edge> & edges;
    const std::vector<int> & c;

public:

    typedef T Value;
    typedef lemon::ListGraph::Edge Key;

    Value operator[](Key e) const {
    // return orig_len[e]-(pot[g.target(e)]-pot[g.source(e)]);
    
    for (int i=0; i<c.size(); i++) {
        if (e == edges[i]) {
            return 0;
        }
    }
    return secondStageCosts[e];
  }
  
    SecondStageMap(const lemon::ListGraph::EdgeMap<T> & s, const std::vector<lemon::ListGraph::Edge> & e, const std::vector<int> & _c)
    : secondStageCosts(s), edges(e), c(_c) {
    }

};

template void bruteForceEnumeration<double> (const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::reference_wrapper<lemon::ListGraph::EdgeMap<double>>> & scenarioSecondStageCosts);
template int twoStageSetting<int>(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<int> & firstStageCosts, const lemon::ListGraph::EdgeMap<int> & secondStageCosts, bool save);