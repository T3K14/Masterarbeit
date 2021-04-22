#pragma once
#include <random>
#include <vector>
#include <lemon/list_graph.h>
std::vector<double> calcScenarioProbabilities(size_t n, std::mt19937 & rng);

// void writeGraphToLGF(const lemon::ListGraph & g);    // brauche ich gar nicht unbedingt, da ich die maps auch uebergeben muesste und das nicht wirklich Schreibaufwand spart


void inittwiddle(int m, int n, std::vector<int> & p);           //could also be function template that uses an array of size typename size

bool twiddle(int & x, int & y, int & z, std::vector<int> & p);  //could also be function template that uses an array of size typename size
