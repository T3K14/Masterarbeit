#include <vector>

#include "../two_stage_gurobi.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"
#include "../simulate.hpp"

#include <iostream>
#include <fstream>
#include <map>

using namespace std;

/* 
Dieses Skript ist dazu da, um zu testen, wieso der LP Alg so langsam war (exponentiell langsamer wurde)
*/

// input argumente: number_edges, number_scenarios 
int main(int argc, char * argv[]) {

    int number_edges = stoi(argv[1]);
    int number_scenarios = stoi(argv[2]);
    std::string directory(argv[3]);

    // int iterations = stoi(argv[2]);

    // if (number_nodes < 1 || number_scenarios < 1) {
    //     throw std::invalid_argument( "Anzahl Knoten und Anzahl Szenarios muessen >1 sein!\n" );
    // }

    // int number_edges = BinomialCoefficient(number_nodes, 2);

    int number_minus_edges;
    int number_nodes;

    if (number_edges > 4950)  {
        throw std::invalid_argument("ROBERT-ERROR, so viele Kanten waeren schon krass..");
    }

    // finde heraus, wie viel Knoten ich brauche
    for (int i=2; i<=100; i++) {
        // fc_edges_to_nodes.insert(std::make_pair(i, BinomialCoefficient(i, 2));)
        // std::cout << BinomialCoefficient(i, 2) << "\n";

        if (number_edges <= BinomialCoefficient(i, 2)) {
            number_nodes = i;
            number_minus_edges = BinomialCoefficient(i, 2) - number_edges;
            break;
        }
    }

    std::string outputPath = R"(/gss/work/xees8992/)";
    outputPath += directory;

    // hier drin speichere ich pro run die Anzahl der hinzugefuegten Bedingungen
    // std::vector<unsigned long> counters;

    // hier drin speichere ich die setupzeiten
    // std::vector<std::chrono::seconds> setup_zeiten;

    // hier drin speichere ich die Loopzeiten (zeit, in der die Bedingungen hinzugefuegt werden und f)
    // std::vector<std::chrono::seconds> loop_zeiten;
    // hier speicher ich die Zeiten, wie lange alle Optimierungen bei allen Iteratioenn gedauert haben
    // std::vector<std::vector<std::chrono::milliseconds>> optimierungs_zeiten;

    try {
       
        // std::cout << "Nodes: " << number_nodes << "\n";
        // std::cout << "Iterationen: " << iterations << "\n";
        // std::cout << "Anzahl Edges: " << number_edges << "\n";

        // loope ueber Anzahl Iterationen und fuere so oft ein fully connected Problem aus und tracke die Dauer und die Anzahl an hinzugefuegten Bedingungen

        // counter
        ofstream counter_file;
        std::string counter_path = outputPath + "/counters.txt";

        // setup zeit
        ofstream setup_file;
        std::string setup_path = outputPath + "/setup_zeiten_s.txt";

        // loop zeit
        ofstream loop_file;
        std::string loop_path = outputPath + "/loop_zeiten_s.txt";

        // die optimierungszeiten
        std::string opt_path = outputPath + "/opt/";

        int counter_iteration = 0;

        NRandomScenarioCreator s(number_scenarios, rng);    
        RandomTestCreator n(0., 10., rng);
        FullyConnectedMinusEdges ensemble(number_nodes, s, n, rng, number_minus_edges);

        while (true) {
        
            unsigned long counter = 0;
            std::chrono::seconds setup_zeit;
            std::chrono::seconds loop_zeit;

            std::vector<std::chrono::milliseconds> opt_times;

            double res = solve_relaxed_lp(ensemble.two_stage_problem, counter, setup_zeit, loop_zeit, opt_times);

            // Ergebnisse abspeichern:

            counter_file.open(counter_path, std::ios::app);
            counter_file << counter << "\n";
            counter_file.close();

            setup_file.open(setup_path, std::ios::app);
            setup_file << setup_zeit.count() << "\n";
            setup_file.close();

            loop_file.open (loop_path, std::ios::app);
            loop_file << loop_zeit.count() << "\n";
            loop_file.close();

            ofstream opt_file;
            opt_file.open(opt_path + std::to_string(counter_iteration) + "_ms.txt");

            for (auto t : opt_times) {
                opt_file << t.count() << "\n";
            }

            opt_file.close();
        
            // naechste Iteration
            counter_iteration++;
            ensemble.recreate();

            // counters.push_back(counter);
            // optimierungs_zeiten.push_back(opt_times);
            // setup_zeiten.push_back(setup_zeit);
            // loop_zeiten.push_back(loop_zeit);

            //std::cout << "Ich komme bis zur Ausgabe des Ergebnisses des LP-Algorithmus\n" << std::endl;


            // mst.approximate(rng);
            // mst.save_approx_result_map("approx_long");

            // double expected_costs = mst.calculate_expected_from_bool_map(mst.approx_first_stage_map);
            // std::cout << "Mit dieser Auswahl hat man Gesamterwartungskosten von : " << expected_costs << std::endl;

            // //output, zum Vergleichen
            // std::cout << expected_costs << std::endl;

        }
        // alles abspeichern

        // // counter
        // ofstream counter_file;
        // std::string counter_path = outputPath + "/counters.txt";

        // std::cout << counter_path << "\n";

        // counter_file.open (counter_path);

        // for (auto c : counters) {
        //     counter_file << c << "\n";
        // }
        // counter_file.close();


        // // setup zeit
        // ofstream setup_file;
        // std::string setup_path = outputPath + "/setup_zeiten_s.txt";

        // setup_file.open (setup_path);

        // for (auto t : setup_zeiten) {
        //     setup_file << t.count() << "\n";
        // }
        // setup_file.close();

        // // loop zeit
        // ofstream loop_file;
        // std::string loop_path = outputPath + "/loop_zeiten_s.txt";

        // loop_file.open (loop_path);

        // for (auto t : loop_zeiten) {
        //     loop_file << t.count() << "\n";
        // }
        // loop_file.close();


        // // die optimierungszeiten
        // std::string opt_path = outputPath + "/opt/";

        // for (int i=0; i<iterations; i++) {
        //     ofstream opt_file;
        //     opt_file.open(opt_path + std::to_string(i) + "_ms.txt");

        //     std::cout << opt_path + std::to_string(i) + "_ms.txt" << "\n";

        //     for (auto t : optimierungs_zeiten[i]) {
        //         opt_file << t.count() << "\n";
        //     }

        //     opt_file.close();
        // }
        
    }
    // catch(GRBException e) {
    //     cout << "Error code = " << e.getErrorCode() << endl;
    //     cout << e.getMessage() << endl;
    // } 
    catch(...) {
        cout << "Exception during optimization" << endl;
    }

    
}