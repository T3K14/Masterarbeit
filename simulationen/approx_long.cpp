#include <vector>

#include "../two_stage_gurobi.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"

using namespace std;

int main() {

    try {
        // sehr grosses Problem mit 10000 Knoten, fully connected (fast 50 Millionen Kanten) mit random kantengewichten aus (0, 10)
        unsigned int number_scenarios = 3;
        unsigned int number_nodes = 10000;
        int number_edges = 49995000;

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

        double res = solve_relaxed_lp(mst);
        mst.save_lp_result_map("lp_long");

        mst.approximate(rng);
        mst.save_approx_result_map("approx_long");

        double expected_costs = mst.calculate_expected_from_bool_map(mst.approx_first_stage_map);
        std::cout << "Mit dieser Auswahl hat man Gesamterwartungskosten von : " << expected_costs << std::endl;

        // //output, zum Vergleichen
        // std::cout << expected_costs << std::endl;
    }
    catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } 
    catch(...) {
        cout << "Exception during optimization" << endl;
    }
}