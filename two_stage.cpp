#include <chrono>
// #include <memory>
#include "two_stage.hpp"
#include "utilities.hpp"
#include <lemon/lgf_writer.h>
#include <lemon/kruskal.h>
#include <array>

#include <lemon/connectivity.h>
#include <lemon/adaptors.h>


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
    //auto costMapPtr = std::make_unique<lemon::ListGraph::EdgeMap<double>>(g);

    // umschreiben fuer cpp11
    std::unique_ptr<lemon::ListGraph::EdgeMap<double>> costMapPtr(new lemon::ListGraph::EdgeMap<double>(g));

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
        lemon::GraphWriter<lemon::ListGraph>(g, outputPath).edgeMap("first_stage", firstStageCosts).edgeMap("second_stage", secondStageCosts).edgeMap("final_selection", minMap).edgeMap("MST_selection", kruskalResMap).run();
        
    }
    return totalCosts;
}


double fourb(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const lemon::ListGraph::EdgeMap<std::vector<double>> & scenarioSecondStageCostsEM) {

    // convert to vector of unique pointers...
    std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> scenarioSecondStageCosts;

    lemon::ListGraph::EdgeIt edg(g);

    for (int i=0; i < scenarioSecondStageCostsEM[edg].size(); i++) {
        //umschreiben, damit es fuer cpp11 laeuft
        std::unique_ptr<lemon::ListGraph::EdgeMap<double>> tmp_unique(new lemon::ListGraph::EdgeMap<double>(g));
        //scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g)); 
        scenarioSecondStageCosts.push_back(std::move(tmp_unique));
        for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {                                 // geht das effizienter mit dem Iterator edg?
            (*scenarioSecondStageCosts[i])[e] = scenarioSecondStageCostsEM[e][i];
        }
    }

    double res = 0.;

    // loope ueber alle szenarios

    for (int i=0; i < scenarioSecondStageCosts.size(); i++) {
        res += scenarioProbabilities[i] * twoStageSetting(g, firstStageCosts, *(scenarioSecondStageCosts[i]), false);
    }

    return res;

}


double bruteForceEnumeration(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const lemon::ListGraph::EdgeMap<std::vector<double>> & scenarioSecondStageCostsEM) {
    // std::cout << "hi\n";

    // convert to vector of unique pointers...
    std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> scenarioSecondStageCosts;

    lemon::ListGraph::EdgeIt edg(g);

    for (int i=0; i < scenarioSecondStageCostsEM[edg].size(); i++) {
        // umschreiben, damit es fuer cpp11 funktioniert
        std::unique_ptr<lemon::ListGraph::EdgeMap<double>> tmp_unique(new lemon::ListGraph::EdgeMap<double>(g));

        //scenarioSecondStageCosts.push_back(std::make_unique<lemon::ListGraph::EdgeMap<double>>(g)); 
        scenarioSecondStageCosts.push_back(std::move(tmp_unique));
        for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {                                 // geht das effizienter mit dem Iterator edg?
            (*scenarioSecondStageCosts[i])[e] = scenarioSecondStageCostsEM[e][i];
        }
    }
    

    return bruteForceEnumeration(g, firstStageCosts, scenarioProbabilities, scenarioSecondStageCosts);

    /*
    size_t NodeCount = lemon::countNodes(g);
    int EdgeCount = lemon::countEdges(g);

    // current best expectation value
    double currentMinEV = 0;

    // brauche ich nicht: ABER ich muss irgendwie die edges fuer den twiddle algorithmus durchnumerieren
    // lemon::ListGraph::EdgeMap<bool> currentBestMST(g);

    // default value is false, so currently no 
    lemon::ListGraph::EdgeMap<bool> currentFirstStageSelection(g, false);           // darin soll immer die aktuell beste Lsg gespeichert werden

    //solution now: create array of edges and use the index of an edge in this array for the twiddle algorithm
    // std::array<lemon::ListGraph::Edge, EdgeCount> edges; 
    std::vector<lemon::ListGraph::Edge> edges(EdgeCount);               // da der Graph in dieser Fkt nicht geaendert wird, steht jede Kante an einem eindeutigen Index

    size_t optCounter = 0;
    for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
        edges[optCounter++] = e;
    }

    // used as temporary output map of the kruskal algorithm
    lemon::ListGraph::EdgeMap<bool> output(g);      

    // counts how many solutions exist that have the same optimum solution
    size_t counter = 1;     

    // first case: no edge is selected in stage 1
    // loop over all scenarios

    for (int i=0; i < scenarioProbabilities.size(); i++) {
        auto weightedResult = scenarioProbabilities[i] * lemon::kruskal(g, *(scenarioSecondStageCosts[i]), output);
        currentMinEV += weightedResult;
    }


    // second case: there are edges selected in the first stage

    double firstStageSum = 0;

    int x,y,z;

    // for every possible number of selectable edges (1,2,3,4,...,NodeCount-1) (if i select more than N-1 in the first stage, I could drop Edges and would still end up with a spanning tree)
    for (int i=1; i<NodeCount; i++) {

        std::vector<int> p(EdgeCount + 2);
        std::vector<int> c(i);       // store the indices of the selected edges for this iteration (hier drin stehen die Indizes zum array "edges", die angeben, welche Kanten in der jeweiligen Iteration in der 1. stage gekauft werden)

        for (int tmp=0; tmp<i; tmp++) {
            c[tmp] = EdgeCount - i + tmp;
        }

        // I want to select i of all [EdgeCount] edges 
        inittwiddle(i, EdgeCount, p);

        // now check the initial case

        // double res = check<T>(c, scenarioProbabilities, scenarioSecondStageCosts, p, output)
        auto res = check(g, c, edges, firstStageCosts, scenarioProbabilities, scenarioSecondStageCosts, output, currentMinEV, optCounter, currentFirstStageSelection);

        while(!twiddle(x, y, z, p)) {
            // update c: 
            c[z] = x;
            res = check(g, c, edges, firstStageCosts, scenarioProbabilities, scenarioSecondStageCosts, output, currentMinEV, optCounter, currentFirstStageSelection);
        }
    }
    std::cout << currentMinEV << " ist die beste Loesung\n";
    return currentMinEV;
    */
}

// deprecated
// template<typename T>        // T is the edge cost type
double bruteForceEnumeration(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> & scenarioSecondStageCosts) {
    // std::cout << "hi\n";

    size_t NodeCount = lemon::countNodes(g);
    int EdgeCount = lemon::countEdges(g);

    // current best expectation value
    double currentMinEV = 0;

    // brauche ich nicht: ABER ich muss irgendwie die edges fuer den twiddle algorithmus durchnumerieren
    // lemon::ListGraph::EdgeMap<bool> currentBestMST(g);

    // default value is false, so currently no 
    lemon::ListGraph::EdgeMap<bool> currentFirstStageSelection(g, false);           // darin soll immer die aktuell beste Lsg gespeichert werden

    //solution now: create array of edges and use the index of an edge in this array for the twiddle algorithm
    // std::array<lemon::ListGraph::Edge, EdgeCount> edges; 
    std::vector<lemon::ListGraph::Edge> edges(EdgeCount);               // da der Graph in dieser Fkt nicht geaendert wird, steht jede Kante an einem eindeutigen Index

    size_t optCounter = 0;
    for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
        edges[optCounter++] = e;
    }

    // used as temporary output map of the kruskal algorithm
    lemon::ListGraph::EdgeMap<bool> output(g);      

    // counts how many solutions exist that have the same optimum solution
    size_t counter = 1;     

    // first case: no edge is selected in stage 1
    // loop over all scenarios

    for (int i=0; i < scenarioProbabilities.size(); i++) {
        auto weightedResult = scenarioProbabilities[i] * lemon::kruskal(g, *(scenarioSecondStageCosts[i]), output);
        currentMinEV += weightedResult;
    }


    // second case: there are edges selected in the first stage

    double firstStageSum = 0;

    int x,y,z;

    // for every possible number of selectable edges (1,2,3,4,...,NodeCount-1) (if i select more than N-1 in the first stage, I could drop Edges and would still end up with a spanning tree)
    for (int i=1; i<NodeCount; i++) {

        std::vector<int> p(EdgeCount + 2);
        std::vector<int> c(i);       // store the indices of the selected edges for this iteration (hier drin stehen die Indizes zum array "edges", die angeben, welche Kanten in der jeweiligen Iteration in der 1. stage gekauft werden)

        for (int tmp=0; tmp<i; tmp++) {
            c[tmp] = EdgeCount - i + tmp;
        }

        // I want to select i of all [EdgeCount] edges 
        inittwiddle(i, EdgeCount, p);

        // now check the initial case

        // double res = check<T>(c, scenarioProbabilities, scenarioSecondStageCosts, p, output)
        auto res = check(g, c, edges, firstStageCosts, scenarioProbabilities, scenarioSecondStageCosts, output, currentMinEV, optCounter, currentFirstStageSelection);

        while(!twiddle(x, y, z, p)) {
            // update c: 
            c[z] = x;
            res = check(g, c, edges, firstStageCosts, scenarioProbabilities, scenarioSecondStageCosts, output, currentMinEV, optCounter, currentFirstStageSelection);
        }
    }
    std::cout << currentMinEV << " ist die beste Loesung\n";
    
    int cou = 0;
    for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
        if (currentFirstStageSelection[e]) {
            cou++;
        }
    }

    std::cout << cou << " Edges werden in Stage 1 gekauft\n"
;
    return currentMinEV;
}
//deprecated
// template <typename T>
double check(const lemon::ListGraph & g, const std::vector<int> & c, const std::vector<lemon::ListGraph::Edge> & edges, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> & scenarioSecondStageCosts, lemon::ListGraph::EdgeMap<bool> & output, double & currentBest, size_t & optCounter, lemon::ListGraph::EdgeMap<bool> & currentFirstStageSelection) {
    // ermittle Kanten, die in stage 1 gekauft werden (ueber c gegeben)
    
    // teilsumme stage 1 berechnen

    double sumEV = 0;
    for (int i : c) {
        sumEV += firstStageCosts[edges[i]];
    }

    // if firststage sum is already larger than the current optimum, save time
    if (sumEV > currentBest) {
        // std::cout << "Unnoetig\n";
        // return crap value to indicate, that this check did not continue
        return -1;
    }

    // loop ueber alle scenarien
    for(int i=0; i<scenarioProbabilities.size(); i++) {
        //setze die kosten von diesen Kanten in der map fuer phase 2 auf 0
        SecondStageMap secondStageMap(*(scenarioSecondStageCosts[i]), edges, c);

        // optimalen mst ausrechnen
        double mst = lemon::kruskal(g, secondStageMap, output);

        // Ergebnis mit Wahrscheinlichkeit gewichten und auf Teilsumme stage 2 addieren
        sumEV += scenarioProbabilities[i] * mst;
    }

        
    // return sumEV;

    // Ergebnis mit bisherigem Optimum speichern, gegebenefalls counter erhoehen und neues bessere Ergebnis speichern, 

    // ZU EPSILONVERGLEICH AENDERN
    if (sumEV == currentBest) {
        optCounter++;
    } else if (sumEV < currentBest) {
        currentBest = sumEV;
        optCounter = 1;
        // gehe ueber alle Kanten und setze die auf 1, die in c drin stehen
        for (auto & e : edges) {
            currentFirstStageSelection[e] = false;
        }

        // NOCHMAL DRUEBER NACHDENKEN, ABER ICH WILL JA DIE, DEREN INDEX IN C DRIN STEHT UND NICHT DIE ERSTEN c KANTEN AUS edges
        // for (int j=0; j<c.size(); j++) {
        for (int j: c) {

            currentFirstStageSelection[edges[j]] = true;
        }
    }

    return sumEV;
}   


double check(const lemon::ListGraph & g, const std::vector<int> & c, const std::vector<lemon::ListGraph::Edge> & edges, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<lemon::ListGraph::EdgeMap<double>> & scenarioSecondStageCosts, lemon::ListGraph::EdgeMap<bool> & output, double & currentBest, size_t & optCounter, lemon::ListGraph::EdgeMap<bool> & currentFirstStageSelection) {
    // ermittle Kanten, die in stage 1 gekauft werden (ueber c gegeben)
    
    // teilsumme stage 1 berechnen
    
    double sumEV = 0;
    for (int i : c) {
        sumEV += firstStageCosts[edges[i]];
    }
    
    // if firststage sum is already larger than the current optimum, save time
    if (sumEV > currentBest) {
        // std::cout << "Unnoetig\n";
        // return crap value to indicate, that this check did not continue
        return -1;
    }

    // loop ueber alle scenarien
    for(int i=0; i<scenarioProbabilities.size(); i++) {
        //setze die kosten von diesen Kanten in der map fuer phase 2 auf 0
        SecondStageMap secondStageMap(scenarioSecondStageCosts[i], edges, c);

        // optimalen mst ausrechnen
        double mst = lemon::kruskal(g, secondStageMap, output);

        // Ergebnis mit Wahrscheinlichkeit gewichten und auf Teilsumme stage 2 addieren
        sumEV += scenarioProbabilities[i] * mst;
    }

        
    // return sumEV;

    // Ergebnis mit bisherigem Optimum speichern, gegebenefalls counter erhoehen und neues bessere Ergebnis speichern, 

    // ZU EPSILONVERGLEICH AENDERN
    if (sumEV == currentBest) {
        optCounter++;
    } else if (sumEV < currentBest) {
        currentBest = sumEV;
        optCounter = 1;
        // gehe ueber alle Kanten und setze die auf 1, die in c drin stehen
        for (auto & e : edges) {
            currentFirstStageSelection[e] = false;
        }

        // NOCHMAL DRUEBER NACHDENKEN, ABER ICH WILL JA DIE, DEREN INDEX IN C DRIN STEHT UND NICHT DIE ERSTEN c KANTEN AUS edges
        // for (int j=0; j<c.size(); j++) {
        for (int j: c) {

            currentFirstStageSelection[edges[j]] = true;
        }
    }

    return sumEV;
    
    // return 2.;
}

SecondStageMap::Value SecondStageMap::operator[](SecondStageMap::Key e) const {
    // return orig_len[e]-(pot[g.target(e)]-pot[g.source(e)]);
    
    // HABE DAS HIER AUCH GEAENDERT!!! So gehe ich ueber alle Indizes in c und schaue, ob die Kanten, die dazu gehoeren der uebergebenen entsprechen
    // for (int i=0; i<c.size(); i++) {
    for (int i: c) {
        if (e == edges[i]) {
            return 0;
        }
    }
    return secondStageCosts[e];
}

// die Klasse speichert alles nur als Referenzen, da wird also nichts kopiert   
SecondStageMap::SecondStageMap(const lemon::ListGraph::EdgeMap<double> & s, const std::vector<lemon::ListGraph::Edge> & e, const std::vector<int> & _c)
    : secondStageCosts(s), edges(e), c(_c) {
    }


// template void bruteForceEnumeration<double> (const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<double> & firstStageCosts, const std::vector<double> & scenarioProbabilities, const std::vector<std::reference_wrapper<lemon::ListGraph::EdgeMap<double>>> & scenarioSecondStageCosts);
template int twoStageSetting<int>(const lemon::ListGraph & g, const lemon::ListGraph::EdgeMap<int> & firstStageCosts, const lemon::ListGraph::EdgeMap<int> & secondStageCosts, bool save);

// die fkt nimmt die Ergebnisse von einer relaxed lp loesung und rundet daraus eine feasible Solution, dabei raus kommt eine Empfehlung, welche Kanten man in der ersten
// Phase kaufen soll (final_first_stage_map)
void TwoStageProblem::approximate(lemon::ListGraph::EdgeMap<std::vector<double>> & result_optimized_values_map, lemon::ListGraph::EdgeMap<bool> & final_first_stage_map, std::mt19937 & rng) {
    
    // hier ist meine Verteilung 
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    // hier ist eine NodeMap, die allen Nodes true zuweist und spaeter fuer den Subgraph benutzt wird, da die aber immer die selbe ist, lasse ich die hier mal global
    lemon::ListGraph::NodeMap<bool> node_map(g, true);


    // habe hier Map, in die wird eine Edge true, falls diese in einer Iteration fuer die erste Stage gekauft wird 
    lemon::ListGraph::EdgeMap<bool> first_stage_edges(g, false);
    
    // das ganze dann sehr gerne parallelisieren

    // so lange, bis alle forrests connected sind (oder eine andere Abbruchbedingung stattfindet) HIER KANN VLLT EIN COUNTER EINGEBAUT WERDEN, DER HOCHZAEHLT, WENN EIN THREAD FERTIG IST
    // UND DAS GANZE LAEUFT SO LANGE, BIS DER COUNTER GLEICH DER ANZAHL AN SZENARIEN IST
    while(true) {

        // jetzt fuer alle scenarios und das kann glaube ich parallelisiert werden
        for (int i=1; i<numberScenarios+1; i++) {
            
            // erstelle auch hier eine EdgeMap, die die second stage Kanten anzeigt
            lemon::ListGraph::EdgeMap<bool> second_stage_edges(g, false);

            // bool connected = false;
            // so lange random Werte ziehen, und die Maps anpassen, bis mein forrest connected ist
            while(true) {
                
                // loop ueber alle Kanten
                for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {

                    // falls die Edge nicht entweder schon in der first stage map oder in meiner second stage Map drin ist
                    if (!first_stage_edges[e] && !second_stage_edges[e]) {

                        // ziehe erste random zahl fuer first stage
                        if (dist(rng) < result_optimized_values_map[e][0]) {

                            first_stage_edges[e] = true;
                        }

                        // koennte jetzt noch checken, dass die Kante nicht gerade in die first_stage_edges aufgenommen wurde, aber ich glaube es ist effizienter auf das if zu verzichten
                        // ich checke hier einfach nach der zweiten random zahl muss (ich denke aber, dass dieser erste check leichter ist, als die random zahl erstellung, in manchen faellen koennte
                        // man so vielleicht etwas zeit sparen, wenn die abfrage im if vor der random zahl abfrage steht und schon das if ungueltig macht)

                        if (dist(rng) < result_optimized_values_map[e][i]) {
                            second_stage_edges[e] = true;
                        }

                    }

                }

                // jetzt checken, ob mein forrest schon connected ist
                lemon::ListGraph::EdgeMap<bool> connected_map(g);
                for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
                    connected_map[e] = first_stage_edges[e] || second_stage_edges[e];
                }
                // weiss nicht, ob ich die Nodemap einfach so als argument erstellen kann
                lemon::SubGraph<lemon::ListGraph> subgraph(g, node_map, connected_map);

                if (lemon::connected(subgraph)) {
                    // ich bin connected und breake raus
                    // HIER UNTER UMSTAENDEN NOCH DEN COUNTER ERHOEHEN!!!
                    break;
                }

                
            }
        }


    }

    // uebertrage noch die Ergebnisse in die output map, die am Ende die Kanten angeben soll, die der Approxmiationsalgorithmus vorschlaegt in der ersten Phase zu kaufen
    for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
        final_first_stage_map[e] = first_stage_edges[e];
    }
    
}

// konstruktor fuer TwoStageProblem
// die number_scenarios lese ich dabei aus dem Vector ab
TwoStageProblem::TwoStageProblem(std::vector<double> & second_stage_probabilites) 
    : numberScenarios(second_stage_probabilites.size()), secondStageProbabilities(second_stage_probabilites), firstStageWeights(g), secondStageWeights(g), lp_results_map(g) {

    }

void TwoStageProblem::save_lp_result_map(std::string output_name, bool on_cluster) {
    if (on_cluster) {
        std::string outputPath(R"(./)");
        outputPath += output_name;
        outputPath += R"(.lgf)";
        lemon::GraphWriter<lemon::ListGraph> writer(g, outputPath); //.edgeMap("lp results", lp_results_map).run();

        // ist jetzt sehr schlecht, aber ich schreibe fuer jedes szenario eine eigene Map, welche ich dann am Ende printe
        std::vector<std::unique_ptr<lemon::ListGraph::EdgeMap<double>>> edge_maps;

        for (int i=0; i<numberScenarios+1; i++) {

            std::unique_ptr<lemon::ListGraph::EdgeMap<double>> tmp_unique(new lemon::ListGraph::EdgeMap<double>(g));
            edge_maps.push_back(std::move(tmp_unique));
            // fuer alle kanten der edgemap des aktuellen szenarios den Wert kopieren
            for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
                (*edge_maps[i])[e] = lp_results_map[e][i];
            }

            writer.edgeMap(std::to_string(i), *edge_maps[i]);
        }

        // AB HIER DEBUGGING
        // std::cout << lp_results_map[g.edgeFromId(0)].size() << std::endl;
        //lemon::ListGraph::EdgeMap<double> firstStage(g);



        //for (lemon::ListGraph::EdgeIt e(g); e!=lemon::INVALID; ++e) {
        //    firstStage[e] = lp_results_map[e][0];
        //}

        //writer.edgeMap("first stage", firstStage);
        // ENDE DEBUGGING
        
        writer.run();
    }
}

FullyConnectedTwoStageMST::FullyConnectedTwoStageMST(unsigned int number_nodes, std::vector<double> & first_stage_weights, std::vector<std::vector<double>> & second_stage_weights, std::vector<double> & scenario_probabilites)
       : TwoStageProblem(scenario_probabilites), numberNodes(number_nodes) {

    // graph erzuegen, in diesem Fall fully connected 
    initialise_graph();

    // weiss nicht, ob der Iterator in der gewuenschten Reihenfolge ueber die Kanten iteriert...
    // The order in which the iterators visit the items is undefined.
    // The only thing you may assume that they will list the items in the same order until the graph is not changed

    // hier uebertrage ich die firststageweights, denn ich gehe davon aus, dass der vector edges so sortiert ist, dass die uebergebenen Werte zu den Kanten passen
    for (int i =0; i<first_stage_weights.size(); i++) {
        firstStageWeights[edges[i]] = first_stage_weights[i]; 
    }

    // jetzt second stage weights in die map mit dem vector uebertragen
    // gehe ueber alle szenarien und dann ueber alle Kanten und uebetrage pro scenario das gewicht in den vector
    for (int s=0; s<numberScenarios; s++) {
        for(int i=0; i<edges.size(); i++) {
            secondStageWeights[edges[i]].push_back(second_stage_weights[s][i]);
        }
    }
}

// Methode zum Erstellen eines Two Stage MST Problems mit einem fully connected graph
void FullyConnectedTwoStageMST::initialise_graph() {
    
    // using lemon::ListGraph::EdgeMap

    //std::array<lemon::ListGraph::Node, numberNodes> node_array;
    //std::vector<lemon::ListGraph::Edge> edge_vector;
    
    // dem Graph die Nodes hinzufuegen
    for (int i=0; i< numberNodes; i++) {
        nodes.push_back(g.addNode());
    }

    // Edges hinzufuegen
    // Dabei ist es jetzt so, dass zuerst alle Kanten durchgegangen werden ab der ersten Node, dann alle ab der zweiten (ohne die Kante zur ersten Node) etc.
    for (int i=0; i<numberNodes; i++) {
        for (int j=0; j<numberNodes; j++) {  // DER INDEX SOLLTE EIGENTLICH AUCH BEI 1 STARTEN KÖNNEN ODER?
            if (i < j) {
                edges.push_back(g.addEdge(nodes[i], nodes[j]));
            }
        }
    }


}
