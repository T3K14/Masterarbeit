#pragma once

#include <vector>
#include <lemon/list_graph.h>
std::vector<double> calcScenarioProbabilities(size_t n);

// void writeGraphToLGF(const lemon::ListGraph & g);    // brauche ich gar nicht unbedingt, da ich die maps auch uebergeben muesste und das nicht wirklich Schreibaufwand spart