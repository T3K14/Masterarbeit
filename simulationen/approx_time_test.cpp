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

    std::string outputPath = R"(/gss/work/xees8992/)";
    outputPath += directory;

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
            setup_zeiten.push_back(setup_zeit);
            loop_zeiten.push_back(loop_zeit);

            //std::cout << "Ich komme bis zur Ausgabe des Ergebnisses des LP-Algorithmus\n" << std::endl;


            // mst.approximate(rng);
            // mst.save_approx_result_map("approx_long");

            // double expected_costs = mst.calculate_expected_from_bool_map(mst.approx_first_stage_map);
            // std::cout << "Mit dieser Auswahl hat man Gesamterwartungskosten von : " << expected_costs << std::endl;

            // //output, zum Vergleichen
            // std::cout << expected_costs << std::endl;

        }
        // alles abspeichern

        // counter
        ofstream counter_file;
        std::string counter_path = outputPath + "/counters.txt";

        std::cout << counter_path << "\n";

        counter_file.open (counter_path);

        for (auto c : counters) {
            counter_file << c << "\n";
        }
        counter_file.close();


        // setup zeit
        ofstream setup_file;
        std::string setup_path = outputPath + "/setup_zeiten_s.txt";

        setup_file.open (setup_path);

        for (auto t : setup_zeiten) {
            setup_file << t.count() << "\n";
        }
        setup_file.close();

        // loop zeit
        ofstream loop_file;
        std::string loop_path = outputPath + "/loop_zeiten_s.txt";

        loop_file.open (loop_path);

        for (auto t : loop_zeiten) {
            loop_file << t.count() << "\n";
        }
        loop_file.close();


        // die optimierungszeiten
        ofstream opt_file;
        std::string opt_path = outputPath + "opt/";\

        for (int i=0; i<iterations; i++) {

            opt_file.open(opt_path + std::to_string(i)) + "_ms.txt";

            for (auto t : optimierungs_zeiten[i]) {
                opt_file << t.count() << "\n";
            }

            opt_file.close();
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