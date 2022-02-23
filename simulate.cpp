#include "simulate.hpp"

// #include <functional>
#include <iostream>
#include <vector>
// #include <set>
#include <map>

#include <lemon/lgf_writer.h>
#include <lemon/kruskal.h>

#include "boost/filesystem.hpp"

// zum testen
#include <chrono>
#include <thread>
// using namespace std::chrono_literals;

using namespace lemon;

// Ensemble::Ensemble() {}

void simulate(unsigned int runs, Ensemble & ensemble, std::set<Alg> & alg_set, const std::string & ordner, bool on_cluster, bool save_problems) {

    // DAS ENSEMBLE MUSS INITIALISIERT UEBERGEBEN WERDEN

    // Namen zu den Algorithmen:
    std::map<Alg, std::string> name_to_alg {{Alg::GreedyApprox, "Greedy"}, {Alg::LPApprox, "LP_Approx"}, {Alg::Optimal, "Optimum"}, {Alg::Schranke4b, "Schranke4b"}};

    if (alg_set.size() < 1) {
        throw std::invalid_argument("ROBERT-ERROR: Brauche mindestens einen Algorithmus, den ich laufen lasse!\n");
    }

    std::string ordner_path;
    // Ordner ist der Ueberordner, in den alle zusammengehoerigen Ordner reinkommen sollen
    if (!on_cluster) {
        ordner_path = "D:\\Uni\\Masterarbeit\\Code\\output\\" + ordner;
    } else {
        ordner_path = "/gss/work/xees8992/" + ordner;
    }

    // erstelle den uebergebenen Ueberordner, falls er noch nicht existiert
    if (!boost::filesystem::exists(ordner_path)) {
        boost::filesystem::create_directory(ordner_path);
    } 

    // dir_path ist dann der Ordner zu den gegebenen Parametern
    // std::string dir_name = std::to_string(runs) + "_runs_" + ensemble.identify_all();
    std::string dir_path = ordner_path + "\\" + ensemble.identify_all();

    unsigned int counter_simulations = 0;
    // checken, ob der Ordner schon existert, wenn ja, zaehle, wie viele identische Simulationen schon gemacht wurden
    if (boost::filesystem::exists(dir_path)) {

        for (boost::filesystem::directory_iterator itr(dir_path); itr != boost::filesystem::directory_iterator(); ++itr) {
            counter_simulations++;
        }
    } else {
        boost::filesystem::create_directory(dir_path);
    }

    // neuen Ordner:    SCHAUEN, OB DAS SO AUCH AUF DEM HPC KLAPPT
    std::string simulation_path = dir_path + "\\simulation_" + std::to_string(counter_simulations);
    boost::filesystem::create_directory(simulation_path);

    // Unterordner fuer die Algorithmen
    for (auto alg: alg_set) {
        boost::filesystem::create_directory(simulation_path + "\\" + name_to_alg[alg]);
    }

    // Unterordner fuer die Problemstellungen
    if (save_problems) {
        boost::filesystem::create_directory(simulation_path + "\\Problems");
    }

    // map, wo die Ergebnisse reingespeichert werden
    std::map<Alg, std::vector<double>> results_map;
    for (auto alg: alg_set) {
        results_map[alg] = std::vector<double>();
    }

    // --- Debug
    std::vector<int> vector_number_edges;
    // --- Ende Debug

    for (int i=0; i<runs; i++) {

        // falls die Problemstellungen gespeichert werden sollen
        if (save_problems) {
            ensemble.save_current_scenarios(simulation_path + "\\Problems", std::to_string(i));
        }

        for (auto alg : alg_set) {

            switch (alg) {
                case Alg::Schranke4b: {
                    double res_4b = ensemble.do4b();
                    results_map[alg].push_back(res_4b);
                }
                break;

                case Alg::LPApprox: {
                // double res_approx = ensemble.approx_after_lp();

                }
                break;

                case Alg::GreedyApprox: {
                    double res = ensemble.greedy();
                    results_map[alg].push_back(res);

                    // speichere die Ergebnismap
                    std::string map_path = simulation_path + "\\" + name_to_alg[alg] + "\\" + std::to_string(i) + ".txt";
                    ensemble.two_stage_problem.save_greedy_first_stage_map(map_path, false);
                }         
                break;
                
                case Alg::Optimal: {
                    double res_optimum = ensemble.bruteforce();
                    results_map[alg].push_back(res_optimum);

                    // speichere die Ergebnismap
                    std::string map_path = simulation_path + "\\" + name_to_alg[alg] + "\\" + std::to_string(i) + ".txt";
                    ensemble.two_stage_problem.save_bruteforce_first_stage_map(map_path, false, false);
                }
                break;                

            // default:
                // break;
            }
        }

        // --Debug
        vector_number_edges.push_back(lemon::countEdges(ensemble.two_stage_problem.g));
        // -- Ende Debug

        // Problemstellung resetten
        ensemble.recreate();
    }
    
    // -- Debug
    std::string filepath_edges = simulation_path + "\\edges_count.txt";
    std::ofstream outFile_edges(filepath_edges, std::ios_base::out);

    if (outFile_edges.is_open()) {

        for (auto edc: vector_number_edges) {
            outFile_edges << std::to_string(edc) + "\n";
        }
    }
    outFile_edges.close();

    // -- Ende Debug

    // alle results abspeichern
    std::string filepath = simulation_path + "\\results.txt";
    std::ofstream outFile(filepath, std::ios_base::out);

    std::vector<Alg> ordered_algs;
    for (auto alg: alg_set) {
        ordered_algs.push_back(alg);
    }

    if (outFile.is_open()) {

        // header schreiben
        std::string header = "";
        for (auto alg: ordered_algs) {
            header += name_to_alg[alg] + ",";
        }

        outFile << header + "\n";

        // einzelne Zeilen schreiben
        for (int i=0; i<runs; i++) {
            std::string line = "";
            for (auto alg: ordered_algs) {
                line += std::to_string(results_map[alg][i]) + ",";                
            }
            outFile << line + "\n";
        }
    }
    else {
        std::cout << "Error beim Fileoeffnen\n";
    }
    outFile.close();
}


/*
void simulate(unsigned int runs, Ensemble & ensemble, Vergleich vergleich) {
    


    // unsigned int successes = 0;

    for(int i=0; i<runs; i++) {
        
        switch (vergleich)
        {
        case Vergleich::ApproxVs4b:
            break;
        case Vergleich::ApproxVsTriv:
            break;
        case Vergleich::ApproxVsBruteforce:

            double approx_res = ensemble.approx_after_lp();
            double optimum_res = ensemble.bruteforce();

            // Bedingung checken, ob das Problem geloest wurde
            // if (approx_res < 1.05 * optimum_res) {
            //     successes++;
            // }

            break;
        
        // default:
            // break;
        }


        // Ensemble neu aufsetzen
        ensemble.recreate();
    }

}
*/

// ueberschreibe die scenario wahrscheinlichkeiten im two_stage_problem
void ScenarioCreator::override_scenarios(TwoStageProblem & tsp, std::vector<double> & scenario_probabilites) {
    tsp.numberScenarios = scenario_probabilites.size();
    tsp.secondStageProbabilities = scenario_probabilites;
}

NRandomScenarioCreator::NRandomScenarioCreator(unsigned int _number_scenarios, std::mt19937 & _rng) : number_scenarios(_number_scenarios), rng(_rng) {}

void NRandomScenarioCreator::create_scenarios(TwoStageProblem & tsp) {

    auto probs = calcScenarioProbabilities(number_scenarios, rng);

    // jetzt ueberschreiben
    override_scenarios(tsp, probs);
}

std::string NRandomScenarioCreator::identify() {
    std::string s = std::to_string(number_scenarios) + "_RandomScenarioCreator";
    return s;
}

RandomTestCreator::RandomTestCreator(double _low, double _high, std::mt19937 & _rng) : low(_low), high(_high), rng(_rng) {

}

std::string RandomTestCreator::identify() {
    std::string s = "RandomTestCreator_" + std::to_string(low) + "_" + std::to_string(high);
    return s;
}

// nimmt die gegebenen neuen Gewichte an und schreibt sie unter ANNAHME, DASS ERSTER EINTRAG IN DEN VEKTOREN SICH AUCH AUF DIE ERSTE KANTE in edges (vektor) bezieht BEZIEHT in das two stage problem
void NewEdgeCostCreator::override_costs(TwoStageProblem & tsp, std::vector<double> & first_stage_costs, std::vector<std::vector<double>> & second_stage_costs) {

    // // ERSTMAL ZUM TESTEN:          WENN ICH DAS AENDERE, DANN AUCH IN RandomTestCreator::create_costs!!!!
    // tsp.numberScenarios = 3;
    // // ENDE

    // uebertrage die neuen Kosten in das two stage problem 
  
    for (int i=0; i<tsp.edges.size(); i++) {
        tsp.firstStageWeights[tsp.edges[i]] = first_stage_costs[i]; 
    }

    // jetzt second stage weights in die map mit dem vector uebertragen
    // gehe ueber alle szenarien und dann ueber alle Kanten und uebetrage pro scenario das gewicht in den vector
    for (int s=0; s<tsp.numberScenarios; s++) {
        for(int i=0; i<tsp.edges.size(); i++) {
            tsp.secondStageWeights[tsp.edges[i]].push_back(second_stage_costs[s][i]);
        }
    }
}

// erzeugt zu den Kanten des twostageproblems random Kosten fuer alle Phasen und Szenarien im Intervall [low, high)
void RandomTestCreator::create_costs(TwoStageProblem & tsp) {

    // // ERSTMAL ZUM TESTEN:          WENN ICH DAS HIER AENDERE, DANN AUCH IN nEWeDGEcOSTcREATOR::override_costs!!!!
    // size_t number_scenarios = 3;
    // // ENDE

    size_t number_edges = tsp.get_number_edges();
    size_t number_scenarios = tsp.get_number_scenarios();

    auto scenarioProbabilities = calcScenarioProbabilities(number_scenarios, rng);
    std::uniform_real_distribution<double> dist(low, high);                

    std::vector<double> first_stage_costs;

    for (size_t i=0; i<number_edges; i++) {
        first_stage_costs.push_back(dist(rng));
    }

    std::vector<std::vector<double>> second_stage_costs;
    for (size_t i=0; i<number_scenarios; i++) {
        std::vector<double> v;
        for (size_t j=0; j<number_edges; j++) {
            v.push_back(dist(rng));
        }
        second_stage_costs.push_back(v);
    } 

    // jetzt rufe ich die overide_costs Methode auf, um die neuen Kosten
    override_costs(tsp, first_stage_costs, second_stage_costs);
}


Ensemble::Ensemble(unsigned int _number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator) : number_nodes(_number_nodes), scenario_creator(_scenario_creator), edge_cost_creator(_edge_cost_creator) {

    // baue schon die Knoten in den Graphen, weil die bleiben erstmal fuer eien Mittelung immer konstant
    for (int i=0; i<number_nodes; i++) {
        two_stage_problem.nodes.push_back(two_stage_problem.g.addNode());
    }
}

void Ensemble::recreate() {
    std::cout << "Ensemble::recreate\n";

     // loesche die bisherigen Kanten
    erase_all_edges();          // SCHAUEN, OB DAS MIT DER VERERBUNG SO KLAPPT
    // fuege neu random so Kanten hinzu, dass ich am Ende einen Tree habe
    add_edges();        // DAS SOLL DANN DIE JEWEILIGE UEBERSCHRIEBENE add_edges METHODE SEIN!!!!!!!!!!!!!!!!!!!!!!!!!

    // fuege neue Scenariowahrscheinlichkeiten hinzu (ensprechend des uebergebenen Scenariocreators)
    scenario_creator.create_scenarios(two_stage_problem);

    //fuege neue Gewichte entsprechend des uebergebenen edgecostCreators hinzu
    edge_cost_creator.create_costs(two_stage_problem);
}

// soll dann eigentlich pure virtual sein!!!!!!!!!!!
// void Ensemble::add_edges() {
//     std::cout << "Ensemble::add_edges\n";
// }

// loesche alle Edges aus dem Graphen raus
void Ensemble::erase_all_edges() {
    for (ListGraph::EdgeIt e(two_stage_problem.g); e != INVALID; ++e) {
        two_stage_problem.g.erase(e);
        
    }
    // ausserdem leere ich den edges vector vom two_stage_problem
    two_stage_problem.edges.clear();
}

void Ensemble::initialize() {

    // rufe die entsprechende add_edges methode auf
    add_edges();

    // fuege die Scenariowahrscheinlichkeiten hinzu ensprechend des uebergebenen Scenariocreators
    scenario_creator.create_scenarios(two_stage_problem);

    //fuege Kantengewichte entsprechend des uebergebenen Edgecoscreators hinzu
    edge_cost_creator.create_costs(two_stage_problem);
}

void Ensemble::save_current_graph(std::string path, std::string name) {

    // std::string path = R"(D:\Uni\Masterarbeit\Code\output\)";
    path += "\\" + name;
    path += R"(.lgf)";

    lemon::GraphWriter<lemon::ListGraph> writer(two_stage_problem.g, path); 

    writer.run();

}

// path ist der Pfad, worein die Szenariodaten gespeichert werden sollen
void Ensemble::save_current_scenarios(std::string path, std::string name) {

    std::string path_scenarios = path + "\\scenarios" + name + ".csv";
    std::string path_scenario_probs = path + "\\scenario_probs" + name + ".csv";


    // speichere die Szenariowahrscheinlichkeiten
    std::ofstream offs;
    offs.open(path_scenario_probs, std::ios_base::out);

    for (auto p: two_stage_problem.secondStageProbabilities) {
        offs << p << "\n";
    }
    offs.close();

    // speichere die Gewichte

    std::ofstream sc;
    sc.open(path_scenarios, std::ios_base::out);

    // header
    sc << "EdgeID, first_stage_costs, ";

    for (int i=0; i<two_stage_problem.numberScenarios; i++) {
        sc << "scenario" << i << ", ";
    }
    sc << "\n";

    // pro Edge die Gewichte

    for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
        
        sc << two_stage_problem.g.id(e) << ", " << two_stage_problem.firstStageWeights[e] << ", ";

        for (int i=0; i<two_stage_problem.numberScenarios; i++) {
            sc << two_stage_problem.secondStageWeights[e][i] << ", ";

        }
        sc << "\n";
    }

    sc.close();


    // speichere noch die Graphstruktur
    // oder ne, dafuer hab ich je extra die andere Fkt.

    // lemon::GraphWriter<lemon::ListGraph> writer(two_stage_problem.g, path_scenarios); 

    // writer.edgeMap("first_stage_costs", two_stage_problem.firstStageWeights);

    // // erstmal so probieren
    // lemon::ListGraph::EdgeMap<double> m(two_stage_problem.g);

    // for (int i=0; i<lemon::countEdges(two_stage_problem.g); i++) {

    //     for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
    //         m[e] = two_stage_problem.secondStageWeights[e][i];
    //     }
    //     std::string name = "szenario" + std::to_string(i);
    //     writer.edgeMap(name, m);
    // }

    // writer.run();


    

}


double Ensemble::bruteforce() {
    return two_stage_problem.bruteforce();
}

void Ensemble::approx_after_lp(std::mt19937 & rng) {
    // nachdem der LP-Alg. fertig ist approximieren
    two_stage_problem.approximate(rng);
}

double Ensemble::greedy() {
    two_stage_problem.greedy();
    // berechne noch den EV, den man mit dieser Loesung erhaelt
    return two_stage_problem.calculate_expected_from_bool_map(two_stage_problem.greedy_first_stage_map);
}

// wahle pro Szenario von jeder Kante die billiger aus 1. und 2. stage aus, baue damit MST und mittle die Gesamtkosten davon ueber alle Szenarien
double Ensemble::do4b() {

    double res = 0.;
    lemon::ListGraph::EdgeMap<double> minMap(two_stage_problem.g); 

    // loope ueber alle szenarien
    for (int i=0; i<two_stage_problem.numberScenarios; i++) {

        // baue fuer dieses Szenario die map mit billigsten Kosten fuer jede Edge
        for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
            minMap[e] = (two_stage_problem.firstStageWeights[e] < two_stage_problem.secondStageWeights[e][i] ? two_stage_problem.firstStageWeights[e] : two_stage_problem.secondStageWeights[e][i]);
        }

        // do the MST calculation
        lemon::ListGraph::EdgeMap<bool> kruskalResMap(two_stage_problem.g);
        double scenario_mst_costs = lemon::kruskal(two_stage_problem.g, minMap, kruskalResMap);

        res += two_stage_problem.secondStageProbabilities[i] * scenario_mst_costs;
    }

    return res;    
}

std::string Ensemble::identify_all() {
    std::string s = identify() + "_" + scenario_creator.identify() + "_" + edge_cost_creator.identify();
    return s;
}

// kann ich den auch noch umschreiben, dass ich nur an den ensemble constructor delegieren muss??? wird dann auch die richtige add_edges methode genommen?
Tree::Tree(unsigned int number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator, std::mt19937 & _rng) : Ensemble(number_nodes, _scenario_creator, _edge_cost_creator), rng(_rng) {
    // delegiere zum Ensemble constructor, der die Knoten erzeugt
    // jetzt kommen noch so viele Kanten dazu, dass das ganze einen Tree ergibt (dieser Vorgang wird dann bei einem recreate-call wiederholt)
}

void Tree::add_edges() {

    std::cout << "Tree::add_edges\n";

    // diese Fkt erwartet, dass der Edgesvektor des Graphen leer ist 

    // gehe so lange alle uebrigen Knoten durch, bis ich alle verbunden hab

    unsigned int remaining_number = number_nodes-1;     // -1, weil die Null selbst schon als linke schranke im Intervall [0, remaining_number] drin ist: wenn ich 4 Knoten zur Auswahl habe, dann muss ich einen Index aus {0,1,2,3} random waehlen und nicht {0,1,2,3,4}
    unsigned int connected_number = 0;
                    
    std::vector<unsigned int> remaining;
    std::vector<unsigned int> connected;

    for (unsigned int i=0; i<number_nodes; i++) {
        remaining.push_back(i);
    }
    
    // random discrete distribution
    std::uniform_int_distribution<unsigned int> dist(0, remaining_number);

    // packe zuefaellig eine Knotennr zu connected
    unsigned int first = dist(rng);
    connected.push_back(first);
    remaining.erase(remaining.begin() + first);

    // brauche ich noch
    unsigned int first_index;
    unsigned int second;
    unsigned int second_index;

    // so lange zweierpaare bilden, bis alle Knoten connected sind
    while (remaining.size() > 0) {

        // passe die Parameter der Verteilung an, um neue der verbleibenden Nodes zu waehlen
        dist.param(decltype(dist)::param_type(0, --remaining_number));

        first_index = dist(rng);   
        first = remaining[first_index];

        // passe die Parameter der Verteilung an, um eine bereits verbundene Node zu waehlen
        dist.param(decltype(dist)::param_type(0, connected_number++));              // ++ als post-increment, damit am Anfang der Index 0 raus kommt
        second_index = dist(rng);
        second = connected[second_index];

        // fuege Kante hinzu
        two_stage_problem.edges.push_back(two_stage_problem.g.addEdge(two_stage_problem.nodes[first], two_stage_problem.nodes[second]));

        // passe remaining und connected an
        remaining.erase(remaining.begin() + first_index);
        connected.push_back(first);
    }
}

std::string Tree::identify() {
    std::string s = "Tree_" + std::to_string(number_nodes) + "_nodes";
    return s;
}

// void Tree::recreate(std::mt19937 & rng) {

//     // loesche die bisherigen Kanten
//     erase_all_edges();          // SCHAUEN, OB DAS MIT DER VERERBUNG SO KLAPPT
//     // fuege neu random so Kanten hinzu, dass ich am Ende einen Tree habe
//     add_edges(rng);

//     // fuege neue Scenariowahrscheinlichkeiten hinzu (ensprechend des uebergebenen Scenariocreators)
//     secenario_creator.create_scenarios(two_stage_problem);

//     //fuege neue Gewichte entsprechend des uebergebenen edgecostCreators hinzu
//     edge_cost_creator.create_costs(two_stage_problem);
// }


TreePlusP::TreePlusP(unsigned int _number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator, std::mt19937 & _rng, double _p)
    : Tree(_number_nodes, _scenario_creator, _edge_cost_creator, _rng), p(_p) {

}

void TreePlusP::add_edges() {

    // erstmal Tree bauen
    Tree::add_edges();

    // -- DEBUG
    // save_current_graph("treeplus0.5_TREE");
    // -- ende DEBUG


    // jetzt jede noch nicht hinzugefuegte Kante mit Wahrscheinlichkeit p hinzufuegen

    // create uniform distribution between 0 and 1
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (int i=0; i<number_nodes; i++) {
        for (int j=i+1; j<number_nodes; j++) {
            
            // falls Kante nicht schon im Tree ist und die Wahrscheinlichkeit trifft
            if (lemon::findEdge(two_stage_problem.g, two_stage_problem.nodes[i], two_stage_problem.nodes[j]) == lemon::INVALID && dist(rng) < p) {
                // fuege Kante hinzu
                two_stage_problem.edges.push_back(two_stage_problem.g.addEdge(two_stage_problem.nodes[i], two_stage_problem.nodes[j]));
            }
        }
    }

    // for (lemon::ListGraph::NodeIt n(g); n!=lemon::INVALID; ++n) {    }

}

std::string TreePlusP::identify() {
    std::string s = "TreePlusP_" + std::to_string(p) + "_p_" + std::to_string(number_nodes) + "_nodes";
    return s;
}

// NOCH NICHT FERTIG
TreePlusEdges::TreePlusEdges(unsigned int _number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator, std::mt19937 & _rng, unsigned int _number_extra_edges) 
    : Tree(_number_nodes, _scenario_creator, _edge_cost_creator, _rng), number_extra_edges(_number_extra_edges) {

    // checke, ob es mit der geforderten Anzahl an Extraedges zu Problemen kommt
    if (number_nodes - 1 + number_extra_edges > BinomialCoefficient(number_nodes, 2)) {         //ACHTUNG WENN ICH UNSIGNED INTS VONEINANDER ABZIEHE
        throw std::invalid_argument( "ROBERT-ERROR: The number of extra edges plus the number of the edges in the tree canvnot be bigger than the number of edges in a fully connected graph!\n" );
    }

}

// NOCH NICHT FERTIG
void TreePlusEdges::add_edges() {

    // zuerst den Baum bauen
    Tree::add_edges();

    //

     // random discrete distribution
    // std::uniform_int_distribution<unsigned int> dist(0, 10);

}

FullyConnected::FullyConnected(unsigned int _number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator) : Ensemble(_number_nodes, _scenario_creator, _edge_cost_creator) {
    // delegiere zum Ensemble constructor, der die Knoten erzeugt
}

void FullyConnected::add_edges() {

    // Dabei ist es jetzt so, dass zuerst alle Kanten durchgegangen werden ab der ersten Node, dann alle ab der zweiten (ohne die Kante zur ersten Node) etc.
    for (int i=0; i<number_nodes; i++) {
        for (int j=0; j<number_nodes; j++) {  // DER INDEX SOLLTE EIGENTLICH AUCH BEI 1 STARTEN KÖNNEN ODER?
            if (i < j) {
                two_stage_problem.edges.push_back(two_stage_problem.g.addEdge(two_stage_problem.nodes[i], two_stage_problem.nodes[j]));
            }
        }
    }

}

std::string FullyConnected::identify() {
    std::string s = "FullyConnected";
    return s;
}

FullyConnectedMinusEdges::FullyConnectedMinusEdges(unsigned int _number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator, std::mt19937 & _rng, unsigned int _number_minus_edges)
    : FullyConnected(_number_nodes, _scenario_creator, _edge_cost_creator), rng(_rng), number_minus_edges(_number_minus_edges) {

    // checken, dass ich nicht so viele Kanten wegnehme, dass am Ende kein Tree mehr vorhanden sein kann
    if (BinomialCoefficient(number_nodes, 2) < number_minus_edges || BinomialCoefficient(number_nodes, 2) - number_minus_edges < number_nodes - 1) {     // erste Bedingung, weil ich hier mit unsigned ints hantiere
        throw std::invalid_argument("ROBERT-ERROR: The number of remaining edges must be bigger than N-1.\n" );
    }
}

void FullyConnectedMinusEdges::add_edges() {

    // diese Methode koennte save noch optimiert werden

    // erstmal fully connected Edges bauen
    FullyConnected::add_edges();

    // vector mit edges, die ich noch entfernen darf
    std::vector<lemon::ListGraph::Edge> edges_left(two_stage_problem.edges);

    unsigned int number_edges_left = two_stage_problem.edges.size();
        
    std::uniform_int_distribution<unsigned int> dist(0, --number_edges_left);       // --, weil uniform int dist aus Intervall [a,b] zieht

    for (int i=0; i<number_minus_edges; i++) {

        // passe die Parameter der Verteilung an, um neue Edge aus den verbleibenden zu waehlen
        dist.param(decltype(dist)::param_type(0, number_edges_left--));

        // waehle random eine edge aus und entferne sie
        unsigned int index = dist(rng);
        auto e = edges_left[index];

        edges_left.erase(edges_left.begin() + index);

        // auch noch aus dem two_stage_problem entfernen, zuerst aus edges und dann aus dem Graph
        // finde zuerst den Index der Edge in two_stage_problem.edges
        auto iter = std::find(two_stage_problem.edges.begin(), two_stage_problem.edges.end(), e);
        // int index = std::distance(two_stage_problem.edges.begin(), iter);

        two_stage_problem.edges.erase(two_stage_problem.edges.begin() + std::distance(two_stage_problem.edges.begin(), iter));
        
        two_stage_problem.g.erase(e);

        // aktualisiere die edges_left indem alle entfernt werden, die nun, nachdem die neue Edge entfernt wurde nicht mehr entfernt werden duerfen

        auto it = edges_left.begin();

        while(it != edges_left.end()) {

            // zur edge: *it 
            // schauen, ob ein Knoten nun nur noch 1 inzidente Kante hat
            if(lemon::countIncEdges(two_stage_problem.g, two_stage_problem.g.u(*it)) < 2 || lemon::countIncEdges(two_stage_problem.g, two_stage_problem.g.v(*it)) < 2) {

                it = edges_left.erase(it);
            }
            else ++it;
        }
    }
}

std::string FullyConnectedMinusEdges::identify() {
    std::string s = "FullyConnectedMinus_" + std::to_string(number_minus_edges) + "_edges";
    return s;
}


/*
int main() {

    /
    ListGraph g;
    const unsigned nodeNumber = 3;
    std::vector<ListGraph::Node> nodes;
    // std::array<ListGraph::Node, nodeNumber> nodes;

    for(int i=0; i < nodeNumber; i++) {
        nodes.push_back(g.addNode());
    }

    // std::array<ListGraph::Edge, 11> edges;
    std::vector<ListGraph::Edge> edges;

    edges.push_back(g.addEdge(nodes[0], nodes[1]));
    edges.push_back(g.addEdge(nodes[0], nodes[2]));
    edges.push_back(g.addEdge(nodes[1], nodes[2]));

    for (ListGraph::EdgeIt e(g); e != INVALID; ++e) {
        g.erase(e);
    }

    std::cout << countEdges(g) << "\n";
    /

    // -----------------------------

    auto t1 = std::chrono::high_resolution_clock::now();


    Tree tree(100, rng);
    tree.save_current_graph("tree");


    // std::this_thread::sleep_for(2000ms);


    auto t2 = std::chrono::high_resolution_clock::now();
    auto s_int = std::chrono::duration_cast<std::chrono::seconds>(t2-t1);
    std::cout << "duration: " << s_int.count() << "s\n";

    return 0;
}
*/