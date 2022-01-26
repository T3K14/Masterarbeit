#include "simulate.hpp"

// #include <functional>
#include <iostream>
#include <vector>

#include <lemon/lgf_writer.h>

// zum testen
#include <chrono>
#include <thread>
// using namespace std::chrono_literals;

using namespace lemon;

// Ensemble::Ensemble() {}
/*
void simulate(unsigned int runs, Ensemble & ensemble, Vergleich vergleich) {
    
    unsigned int successes = 0;

    for(int i=0; i<runs; i++) {
        
        switch (vergleich)
        {
        case Vergleich::ApproxVs4b:
            break;
        case Vergleich::ApproxVsTriv:
            break;
        case Vergleich::ApproxVsBruteforce:

            double approx_res = ensemble.approx();
            double optimum_res = ensemble.bruteforce();

            // Bedingung checken, ob das Problem geloest wurde
            if (approx_res < 1.05 * optimum_res) {
                successes++;
            }

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

RandomTestCreator::RandomTestCreator(double _low, double _high, std::mt19937 & _rng) : low(_low), high(_high), rng(_rng) {

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
    std::uniform_real_distribution<double> dist(0., 10.);                

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


Ensemble::Ensemble(unsigned int _number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator) : number_nodes(_number_nodes), secenario_creator(_scenario_creator), edge_cost_creator(_edge_cost_creator) {

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
    secenario_creator.create_scenarios(two_stage_problem);

    //fuege neue Gewichte entsprechend des uebergebenen edgecostCreators hinzu
    edge_cost_creator.create_costs(two_stage_problem);
}

// soll dann eigentlich pure virtual sein!!!!!!!!!!!
void Ensemble::add_edges() {
    std::cout << "Ensemble::add_edges\n";
}

// loesche alle Edges aus dem Graphen raus
void Ensemble::erase_all_edges() {
    for (ListGraph::EdgeIt e(two_stage_problem.g); e != INVALID; ++e) {
        two_stage_problem.g.erase(e);
        
    }
    // ausserdem leere ich den edges vector vom two_stage_problem
    two_stage_problem.edges.clear();
}

// kann ich den auch noch umschreiben, dass ich nur an den ensemble constructor delegieren muss??? wird dann auch die richtige add_edges methode genommen?
Tree::Tree(unsigned int number_nodes, ScenarioCreator & _scenario_creator, NewEdgeCostCreator & _edge_cost_creator, std::mt19937 & _rng) : Ensemble(number_nodes, _scenario_creator, _edge_cost_creator), rng(_rng) {
    // delegiere zum Ensemble constructor, der die Knoten erzeugt
    // jetzt kommen noch so viele Kanten dazu, dass das ganze einen Tree ergibt (dieser Vorgang wird dann bei einem recreate-call wiederholt)
    add_edges();

    // fuege die Scenariowahrscheinlichkeiten hinzu ensprechend des uebergebenen Scenariocreators
    secenario_creator.create_scenarios(two_stage_problem);

    //fuege Kantengewichte entsprechend des uebergebenen Edgecoscreators hinzu
    edge_cost_creator.create_costs(two_stage_problem);
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

void Ensemble::save_current_graph(std::string name) {

    std::string path = R"(D:\Uni\Masterarbeit\Code\output\)";
    path += name;
    path += R"(.lgf)";

    lemon::GraphWriter<lemon::ListGraph> writer(two_stage_problem.g, path); 

    writer.run();

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