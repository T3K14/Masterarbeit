#include "utilities.hpp"
#include <unordered_map>
// #include <ctime>

// #include <iostream>

// define the RNG
std::mt19937 rng{std::random_device{}()};               // construct it with the seed that I get from the random_device rvalue on which I use the ()-operator (functor)


std::vector<double> calcScenarioProbabilities(size_t n, std::mt19937 & rng) {

    // const auto seed = time(nullptr);
    // std::mt19937 eng(static_cast<std::mt19937::result_type>(seed));

    std::uniform_real_distribution<double> dist(1, 1000);                // Intervall anders machen? Dazu gibts schriftliche Ueberlegungen vom 15.04.21

    double sum = 0;
    std::vector<double> vec;
    vec.reserve(n);
    for (int i=0; i<n; i++) {
        double res = dist(rng);
        vec.push_back(res);
        sum += res;
    }

    for (int i=0; i<n; i++) {
        vec[i] /= sum;
    }

    return vec;
} 

// int main() {

//     double n = 5;
//     auto res = calcScenarioProbabilities(n);
//     for (int i=0; i<n; i++) {
//         std::cout << res[i] << ", ";
//     }
//     std::cout << "\n";
// }

void inittwiddle(int m, int n, std::vector<int> & p) {
  int i;
  p[0] = n+1;
  for(i = 1; i != n-m+1; i++)
    p[i] = 0;
  while(i != n+1)
    {
    p[i] = i+m-n;
    i++;
    }
  p[n+1] = -2;
  if(m == 0)
    p[1] = 1;
  }

bool twiddle(int & x, int & y, int & z, std::vector<int> & p) {

    int i, j, k;
    j = 1;
    while(p[j] <= 0)
        j++;
    if(p[j-1] == 0) {
        for(i = j-1; i != 1; i--)
            p[i] = -1;
        p[j] = 0;
        x = z = 0;
        p[1] = 1;
        y = j-1;
    }
    else {
        if(j > 1)
            p[j-1] = 0;
        do
            j++;
        while(p[j] > 0);
        k = j-1;
        i = j;
        while(p[i] == 0)
            p[i++] = -1;
        if(p[i] == -1) {
            p[i] = p[k];                // lande ich hier auch bei p[k] = 2 oder noch groesser?
            z = p[k]-1;
            x = i-1;
            y = k-1;
            p[k] = -1;
        }
        else {
            if(i == p[0])
	            return(1);
            else {
	            p[j] = p[i];
	            z = p[i]-1;
	            p[i] = 0;
	            x = j-1;
	            y = i-1;
	        }
        }
    }
    return(0);
}

std::pair<std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>>, std::vector<double>> edgeWeightIncrease(const lemon::ListGraph & g ,const lemon::ListGraph::EdgeMap<double> & firstStageWeights, double p, double factor, size_t N, std::mt19937 & rng) {

    //-------------------


    // std::vector<lemon::ListGraph::EdgeMap<double>> testVec;
    // // testVec.push_back();

    // lemon::ListGraph::EdgeMap<double> map2(g);
    // testVec.push_back(map2);




    //-------------------



    // annahme, dass sich die Kantenreihenfolge nicht anedert, weil Graph auch nicht geaendert wird

    std::pair<std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>>, std::vector<double>> pair;
    std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> & secondStageCostScenarios = pair.first;
    std::vector<size_t> scenarioCounts;
    
    // map, wo zur Anzahl an Aenderungen die Szenarien gespeichert sind
    // std::unordered_map<unsigned int, std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>>>  helpMap;

    std::unordered_map<unsigned int, std::vector<std::pair<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>, unsigned int>>> helpMap;
    // std::vector<std::pair<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>, unsigned int>>
    // create uniform distribution between 0 and 1
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    // size_t uniqueCounter = 0;

    // Sample of N scenarios
    for (int i=0; i<N; i++) {

        unsigned int counter = 0; // counts how many changes have been made

        // create new scenario
        lemon::ListGraph::EdgeMap<double> map(g);
        
        // iteriere ueber alle Edges
        for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {

            // mit Wahrscheinlichkeit p Kantengewicht anpassen
            if (dist(rng) < p) {                                    // KLEINER ODER KLEINER GLEICH???

                map[e] = firstStageWeights[e] * factor;
                // mit counter tracken, wie viele Aenderungen vorgenommen wurden
                counter++;

            } else {
                map[e] = firstStageWeights[e];
            }
        }

        // schauen, ob dieses Szenario schon existiert

        // dazu iteriere ich ueber alle Szenarien, die gleich viele Aenderungen haben und elementwises Vergleichen
        size_t identicalIndex;  // If there is an identical scenario among the already created ones, its index will be stored here   
        bool globalSame = false;    // wenn das am Ende noch falsch ist, gibt es kein identisches Szenario
        for (auto & scenarioPair : helpMap[counter]) {
            
            // same says if both edgeMaps represent the same scenario
            bool same = true;
            lemon::ListGraph::EdgeIt e(g);
            while(same && e != lemon::INVALID) {
                if ((*scenarioPair.first)[e] != map[e]) {
                    same = false;
                    // break;      // break, weil sobald ein Gewicht anders ist, koennen die szenarien nicht mehr gleich sein   // DAS REGELT SAME IM WHILE SCHON
                }

                ++e;
            }

            // at this point same == true means, that the current scenario 'scenarioPair.first' is equal to the newly generated scenario 'map'


            if (same) {
                globalSame = true;
                identicalIndex = scenarioPair.second;
                // break if same == true, weil es kein weiteres gleiches Szenario geben kann und es sich daher nicht lohnt weiter zu iterieren
                break;
            }

        }

        if (globalSame) {
            // increase the count for calculating the probability later
            scenarioCounts[identicalIndex]++;
        } else {
            // secondStageCostScenarios.push_back(std::unique_ptr<lemon::ListGraph::EdgeMap<double>>(&map));    //SO WARS VORHER
            secondStageCostScenarios.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));

            for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
                // uebertrage die Gewichte auf die gerade hinzugefuegte EdgeMap 
                (*secondStageCostScenarios[secondStageCostScenarios.size()-1])[e] = map[e];
            }




            scenarioCounts.push_back(1);    // for calculating the probability

            // also add it to the map, damit folgende Szenarien sich auch mit diesem Vergleichen koennen
            // secondStageCostScenarios.size() sagt mir, wie viele einzigartige scenarios schon gespeichert sind und damit auch, welchen Index, mein Szenario in dieser Liste hat
            // helpMap[counter].push_back(std::make_pair(std::unique_ptr<lemon::ListGraph::EdgeMap<double>>(&map), secondStageCostScenarios.size()-1));     //SO WARS BISHER
            helpMap[counter].push_back(std::make_pair(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g), secondStageCostScenarios.size()-1));  

            for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
                (*secondStageCostScenarios[secondStageCostScenarios.size()-1])[e] = map[e];
                // uebertrage die Gewichte auf die gerade hinzugefuegte EdgeMap 

                // (*helpMap[counter].back().first())[e] = map[e];
                (*helpMap[counter].back().first)[e] = map[e];
                // auto & a = helpMap[counter].back().first();

            }

        }


    }

    // std::vector<double> scenarioProbabilities(scenarioCounts.size());
    for (int i=0; i< scenarioCounts.size(); i++) {
        pair.second.push_back(scenarioCounts[i] / N);
    }
    return pair;
}

