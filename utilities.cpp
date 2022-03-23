#include "utilities.hpp"
#include <set>
#include <utility>
#include <lemon/core.h>

#include <cassert>

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

/*
std::pair<TestStruct, TestStruct> edgeWeightIncrease(const lemon::ListGraph & g ,const lemon::ListGraph::EdgeMap<double> & firstStageWeights, double p, double factor, size_t N, std::mt19937 & rng) {

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
    return std::make_pair(TestStruct{2}, TestStruct{});
}
*/
// std::vector<double> edgeWeightIncrease2(lemon::ListGraph::EdgeMap<std::vector<double>> & edgMap, const lemon::ListGraph & g ,const lemon::ListGraph::EdgeMap<double> & firstStageWeights, double p, double factor, int N, std::mt19937 & rng) {
// std::pair<lemon::ListGraph::EdgeMap<std::vector<double>>, std::vector<double>> edgeWeightIncrease2(const lemon::ListGraph & g ,const lemon::ListGraph::EdgeMap<double> & firstStageWeights, double p, double factor, int N, std::mt19937 & rng) {

// erklaerung dazu, was die Fkt macht, gibt es fuer mich im Latexdokument, Da steht auch, dass ich denke, dass ich die Fkt vllt. noch besser implementieren koennte!!!
void edgeWeightIncrease(lemon::ListGraph::EdgeMap<std::vector<double>> & edgMap, std::vector<double> & scenarioProbabilities, const lemon::ListGraph & g ,const lemon::ListGraph::EdgeMap<double> & firstStageWeights, double p, double factor, int N, std::mt19937 & rng) {
    // annahme, dass sich die Kantenreihenfolge nicht anedert, weil Graph auch nicht geaendert wird

    // std::vector<lemon::ListGraph::EdgeMap<double>> vec);


    using vectorEdgeMap = lemon::ListGraph::EdgeMap<std::vector<double>>;
    // std::pair<vectorEdgeMap, std::vector<double>> pair(vectorEdgeMap(g), std::vector<double>);          // FUNKTIONIERT DIE INITIALISIERUNG SO? NEIN, denn c++ denkt, es ist fkts prototyp

    // count how often unique scenarios occur
    std::vector<int> scenarioCounts;
    
    // map, wo zur Anzahl an Aenderungen die Szenarien gespeichert sind.
    // benutze pairs, wo der erste Eintrag das Kantengewicht für das jeweilige Szenario enthaelt und der zweite Eintrag den Index angibt, unter welchem das Szenario in 'edgMap' gespeichert wird
    std::unordered_map<int, lemon::ListGraph::EdgeMap<std::vector<std::pair<double, int>>>> helpMap;
 
    // create uniform distribution between 0 and 1
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    // Sample of N scenarios
    for (int i=0; i<N; i++) {

        int counter = 0; // counts how many changes have been made

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
                map[e] = firstStageWeights[e]; /// factor;         // DAS '/factor' IST ERSTMAL NUR ZUM TESTEN DA
            }
        }

        // schauen, ob dieses Szenario schon existiert

        // dazu iteriere ich ueber alle Szenarien, die gleich viele Aenderungen haben und Vergleiche diese dann elementwise
        int identicalIndex;  // If there is an identical scenario among the already created ones, its index will be stored here   
        // bool globalSame = true;    // wenn das am Ende noch true ist, gibt es ein identisches Szenario
        lemon::ListGraph::EdgeIt e(g);

        // hier schaue ich, ob es schon eine EdgeMap zu der anzahl aenderungen gibt. Falls nicht, fuege ich den hinzu
        // falls es schon einen gibt, ist alles gut, falls nicht, feuege ich es hier zu dem counter hinzu und muss es dann spaeter nicht nochmal machen
        if (helpMap.find(counter) == helpMap.end()) {
            helpMap.emplace(counter, g);    // counter als pair.first und g als argument fuer den constructor von der edgemap pari.second
        }
        auto numberMatchingScenarios = helpMap.at(counter)[e].size();       // number of scenarios that already exist and have the same amount of changes

        // set in dem alle moeglichen szenarien mit 'counter' Aenderungen per Index gespeichert werden und welche entfernt werden, wenn klar ist, dass sie nicht identisch mit 'map' sind
        std::set<int> s;
        for (int setCounter=0; setCounter < numberMatchingScenarios; setCounter++) {
            s.insert(s.end(), setCounter);      //s.end() ist iterator, der auf das past-the-end element zeigt
        }
        // so lange, es noch szenarien gibt, bei denen bisher alle Kantengewichte mit der neu erstellten map uebereinstimmen, vergleiche ich so lange die naechste Kante, bis entweder keins
        // mehr uebrig ist, oder alle Kanten verglichen sind. In letzterem Fall, existiert das Szenario bereits
        while(!s.empty() && e != lemon::INVALID) {       

            std::vector<int> toDelete;

            // loop over all remaining scenarios
            for (int j : s) {
                
                if (helpMap.at(counter)[e][j].first != map[e]) {

                    toDelete.push_back(j);
                }
            }
            for (int j : toDelete) {
                s.erase(j);
            }
            ++e;
        }
        
        /*
        // entweder ist hier das Set leer oder noch ein Szenario uebrig
        // if (s.empty()) {
        //     globalSame = false;
        // } else {
        //     // setze 'identicalIndex' auf den Index des identischen Szenarios
        //     int index = *(s.begin());
        //     lemon::ListGraph::EdgeIt e(g);                                      //???
        //     identicalIndex = helpMap.at(counter)[e][index].second;
        //     // identicalIndex = helpMap.at(counter)[e][index].second;
        // }
        
        // if (globalSame) {
        //     // increase the count for calculating the probability later
        //     scenarioCounts[identicalIndex]++;
        // } else {
        //     // fuege 'edgMap' das neue Szenario hinzu
        //     // secondStageCostScenarios.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));

        //     for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
        //         // uebertrage die Gewichte auf die gerade hinzugefuegte EdgeMap 
        //         edgMap[e].push_back(map[e]);

        //         // also add it to the map, damit folgende Szenarien sich auch mit diesem Vergleichen koennen
        //         // falls es zu diesem Counter noch keine EdgeMap gibt, emplace sie in die unordered 'helpMap'
        //         if (helpMap.find(counter) == helpMap.end()) {

        //             helpMap.emplace(counter, g);
        //             // helpMap.emplace(std::make_pair(counter, g));
        //             // helpMap.emplace(std::piecewise_construct, std::forward_as_tuple(counter), std::forward_as_tuple(g));
        //         } 

        //         helpMap.at(counter)[e].push_back(std::make_pair(map[e], edgMap[e].size()-1));       // pair.first[e].size() sagt mir, wie viele einzigartige scenarios schon gespeichert sind und damit auch, welchen Index, mein Szenario in dieser Liste hat
        //         // helpMap.at(counter)[e].push_back(map[e]);
        //     }
        //     scenarioCounts.push_back(1);    // for calculating the probability
        
        // }
        */

        if (s.empty()) {
            // fuege 'edgMap' das neue Szenario hinzu
            // secondStageCostScenarios.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g));

            for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
                // uebertrage die Gewichte auf die gerade hinzugefuegte EdgeMap 
                edgMap[e].push_back(map[e]);

                // also add it to the map, damit folgende Szenarien sich auch mit diesem Vergleichen koennen

                // ########## muss ich hier nicht mehr machen!
                // // falls es zu diesem Counter noch keine EdgeMap gibt, emplace sie in die unordered 'helpMap'
                // if (helpMap.find(counter) == helpMap.end()) {

                //     helpMap.emplace(counter, g);
                //     // helpMap.emplace(std::make_pair(counter, g));
                //     // helpMap.emplace(std::piecewise_construct, std::forward_as_tuple(counter), std::forward_as_tuple(g));
                // } 

                helpMap.at(counter)[e].push_back(std::make_pair(map[e], edgMap[e].size()-1));       // pair.first[e].size() sagt mir, wie viele einzigartige scenarios schon gespeichert sind und damit auch, welchen Index, mein Szenario in dieser Liste hat
                // helpMap.at(counter)[e].push_back(map[e]);
            }
            scenarioCounts.push_back(1);    // for calculating the probability
        } else {        // there exists one identical scenario, so find it and increase its counter. No new scenario will be created
            int index = *(s.begin());
            // nehme hier einfach die Edge mit ID 0
            auto ed = g.edgeFromId(0);
            // lemon::ListGraph::EdgeIt e(g);                                      //??? ist schon bloed, sich hier nochmal eine Kante auszudenken
            identicalIndex = helpMap.at(counter)[ed][index].second;
            scenarioCounts[identicalIndex]++;
        }
    }
    
    // std::vector<double> scenarioProbabilities(scenarioCounts.size());
    for (int i=0; i< scenarioCounts.size(); i++) {
        // pair.second.push_back(scenarioCounts[i] / N);
        scenarioProbabilities.push_back(scenarioCounts[i] / (double) N);
    }
    // return pair;
    // return std::make_pair(pair.first, pair.second);
    // return std::make_pair(vectorEdgeMap(g), scenarioProbabilities);
}

int BinomialCoefficient(const int n, const int k) {
  //assert(n >= k);
  //assert(k >= 0);

  if (k == 0) {
    return 1;
  }

  // Recursion ->  (n, k) = (n - 1, k - 1) * n / k
  int step1 = n - k + 1;
  int step0;
  for (int i = 1; i < k; ++i) {
    step1 = (step0 = step1) * (n - k + 1 + i) / (i + 1);
  }

  return step1;
}


void update_c(std::vector<int> & c, const int & number_edges, const int & number_nodes, const bool & stop) {
    // update_c wird am Ende des loops ausgefuert, also ist es so, dass wenn diese Funktion aufgerufen wird, das c, was hier uebergeben wird bereits gecheckt wurde 
    // und nun das naechste ausgerechnen werden muss

    // falls ich einen Index hoeher als die Anzahl Kanten habe, entferne ich letzten Index und zaehle den vorherigen um 1 nach oben
    // das ist auch unabhaengig von stop, weil das immer eine leafnode an dem tree ist
    if (c.back() == number_edges-1) {
        c.pop_back();

        // falls c jetzt empty ist, bin ich komplett fertig
        if (c.empty()) {
            return;
        }

        c.back()++;

        return;
    }

    // wenn ich hierher komme und die Laenge von c = N-1 ist, dann weiss ich, das hinten noch nicht der letzte Kantenindex steht und kann einfach den letzten Index in c
    // nach oben zaehlen und returnen, weil ich unabhaengig von stop eine leaf node im Suchbaum erreicht habe
    if (c.size() == number_nodes - 1) {
        c.back()++;
        return;
    }

    // wenn ich stop hab, dann gehe ich den subtree nicht tiefer, sondern nehme mir anstelle der letzten Kante die naechste
    if (stop) {
        c.back()++;
    } else {                // wenn ich keinen stop habe, gehe ich den tree eine stufe tiefer und nehme mir die naechste Kante zusaetzlich dazu
        c.push_back(c.back() + 1);
    }
}

void update_c_new(std::vector<int> & c, const std::vector<int> & gueltig, std::unordered_map<int, int> & gueltig_index_to_edge_index,  const int & number_edges, const int & number_nodes, const bool & stop) {
    // update_c wird am Ende des loops ausgefuert, also ist es so, dass wenn diese Funktion aufgerufen wird, das c, was hier uebergeben wird bereits gecheckt wurde 
    // und nun das naechste ausgerechnen werden muss

    // falls am Ende von c der letzte edge-Index steht, dann entferne ich diesen
    // das ist auch unabhaengig von stop, weil das immer eine leafnode an dem tree ist
    if (c.back() == gueltig.back()) {
        c.pop_back();

        // falls c jetzt empty ist, bin ich komplett fertig
        if (c.empty()) {
            return;
        }

        // ansonsten: erhoehe den jetzt letzten Index auf den naechsten gueltigen
        // das mache ich, indem ich mir den Index von der Kante c.back() in gueltig suche und diesen um 1 erhoehe und damit dann in gueltig den naechsten edge-Index nehme
        c.back() = gueltig[gueltig_index_to_edge_index[c.back()]++];

        return;
    }

    // wenn ich hierher komme und die Laenge von c = N-1 ist, dann weiss ich, das hinten noch nicht der letzte Kantenindex steht und kann einfach den letzten Index in c
    // auf den naechsten gueltigen setzen und returnen, weil ich unabhaengig von stop eine leaf node im Suchbaum erreicht habe
    if (c.size() == number_nodes - 1) {
        c.back() = gueltig[gueltig_index_to_edge_index[c.back()]++];
        return;
    }

    // wenn ich stop hab, dann gehe ich den subtree nicht tiefer, sondern nehme mir anstelle der letzten Kante die naechste
    if (stop) {
        c.back() = gueltig[gueltig_index_to_edge_index[c.back()]++];
    } else {                // wenn ich keinen stop habe, gehe ich den tree eine stufe tiefer und nehme mir die naechste gueltige Kante zusaetzlich dazu
        c.push_back(gueltig[gueltig_index_to_edge_index[c.back()]++]);
    }
}