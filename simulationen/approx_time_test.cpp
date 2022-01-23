#include <vector>

#include "../two_stage_gurobi.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"

#include <iostream>
#include <fstream>

using namespace std;

/* 
Dieses Skript ist dazu da, um zu testen, wieso der LP Alg so langsam war (exponentiell langsamer wurde)
*/

int main(int argc, char * argv[]) {

    // lass ich erstmal auf 3 und schau dann, was das fuer eine Aenderung bewirkt
    unsigned int number_scenarios = 3;


    int number_nodes = stoi(argv[1]);
    int iterations = stoi(argv[2]);

    if (number_nodes < 1 || iterations < 1) {
        throw std::invalid_argument( "Anzahl Knoten und Anzahl Iterationen muessen >1 sein!\n" );
    }


    int number_edges = BinomialCoefficient(number_nodes, 2);

    std::string directory(argv[3]);

    // hier drin speichere ich pro run die Anzahl der hinzugefuegten Bedingungen
    std::vector<unsigned long> counters;

    // hier drin speichere ich die setupzeiten
    std::vector<std::chrono::seconds> setup_zeiten;

    // hier drin speichere ich die Loopzeiten (zeit, in der die Bedingungen hinzugefuegt werden und f)
    std::vector<std::chrono::seconds> loop_zeiten;
    // hier speicher ich die Zeiten, wie lange alle Optimierungen bei allen Iteratioenn gedauert haben
    std::vector<std::vector<std::chrono::milliseconds>> optimierungs_zeiten;

    try {
       
        // std::cout << "Nodes: " << number_nodes << "\n";
        // std::cout << "Iterationen: " << iterations << "\n";
        // std::cout << "Anzahl Edges: " << number_edges << "\n";

        // loope ueber Anzahl Iterationen und fuere so oft ein fully connected Problem aus und tracke die Dauer und die Anzahl an hinzugefuegten Bedingungen

        for (int i=0; i<iterations; i++) {
        
            unsigned long counter = 0;
            std::chrono::seconds setup_zeit;
            std::chrono::seconds loop_zeit;

            std::vector<std::chrono::milliseconds> opt_times;

            // std::vector<double> scenarioProbabilities {0.2, 0.3, 0.5};

            auto scenarioProbabilities = calcScenarioProbabilities(number_scenarios, rng);
            std::uniform_real_distribution<double> dist(0., 10.);                

            std::vector<double> firstStageWeights;

            for (int i=0; i<number_edges; i++) {
                firstStageWeights.push_back(dist(rng));
            }

            std::vector<std::vector<double>> secondStageWeights;
            for (int i=0; i<number_scenarios; i++) {
                std::vector<double> v;
                for (int j=0; j<number_edges; j++) {
                    v.push_back(dist(rng));
                }
                secondStageWeights.push_back(v);
            } 

            // FullyConnectedTwoStageMST mst(number_nodes, firstStageWeights, secondStageWeights, scenarioProbabilities);
            FullyConnectedTwoStageMST mst(number_nodes, firstStageWeights, secondStageWeights, scenarioProbabilities);

            double res = solve_relaxed_lp(mst, counter, setup_zeit, loop_zeit, opt_times);
            // mst.save_lp_result_map("lp_long");

            // Ergebnisse abspeichern:
            counters.push_back(counter);
            optimierungs_zeiten.push_back(opt_times);

            //std::cout << "Ich komme bis zur Ausgabe des Ergebnisses des LP-Algorithmus\n" << std::endl;


            // mst.approximate(rng);
            // mst.save_approx_result_map("approx_long");

            // double expected_costs = mst.calculate_expected_from_bool_map(mst.approx_first_stage_map);
            // std::cout << "Mit dieser Auswahl hat man Gesamterwartungskosten von : " << expected_costs << std::endl;

            // //output, zum Vergleichen
            // std::cout << expected_costs << std::endl;


            // alles abspeichern
            std::cout << "Es gab im ersten Run so viele Bedingungen: " << counters[0] << "\n";

            std::cout << directory << "\n";

            std::string outputPath = R"(/gss/work/xees8992/nodes_)";

            outputPath += std::to_string(number_nodes);
            outputPath += "_iterations_";
            outputPath += std::to_string(iterations);
            outputPath += ".lgf";

            std::cout << outputPath << std::endl;

        }

        
    }
    // catch(GRBException e) {
    //     cout << "Error code = " << e.getErrorCode() << endl;
    //     cout << e.getMessage() << endl;
    // } 
    catch(...) {
        cout << "Exception during optimization" << endl;
    }

    
}