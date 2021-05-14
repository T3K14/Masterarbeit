#pragma once
#include <random>
#include <vector>
#include <lemon/list_graph.h>
std::vector<double> calcScenarioProbabilities(size_t n, std::mt19937 & rng);

// void writeGraphToLGF(const lemon::ListGraph & g);    // brauche ich gar nicht unbedingt, da ich die maps auch uebergeben muesste und das nicht wirklich Schreibaufwand spart

struct TestStruct {

    int field;
};


void inittwiddle(int m, int n, std::vector<int> & p);           //could also be function template that uses an array of size typename size

bool twiddle(int & x, int & y, int & z, std::vector<int> & p);  //could also be function template that uses an array of size typename size

// returntype vllt nochmal anpassen
// std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> edgeWeightIncrease(const lemon::ListGraph::EdgeMap<double> & firstStageWeights, double p, double factor, size_t N);
std::pair<TestStruct, TestStruct> edgeWeightIncrease(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageWeights, double p, double factor, size_t N, std::mt19937 & rng);
std::pair<lemon::ListGraph::EdgeMap<std::vector<double>>, std::vector<double>> edgeWeightIncrease2(const lemon::ListGraph & g ,const lemon::ListGraph::EdgeMap<double> & firstStageWeights, double p, double factor, size_t N, std::mt19937 & rng);

std::vector<double> edgeWeightIncrease(const lemon::ListGraph::EdgeMap<std::vector<double>> & edgMap, const lemon::ListGraph & g ,const lemon::ListGraph::EdgeMap<double> & firstStageWeights, double p, double factor, size_t N, std::mt19937 & rng);
