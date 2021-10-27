#include <vector>

#include "../two_stage_gurobi.hpp"
#include "../two_stage.hpp"
#include "../utilities.hpp"

using namespace std;

int main() {

    try {
        // test 2
        unsigned int numberScenarios = 3;
        unsigned int numberNodes = 6;

        // Problemdefinition

        std::vector<double> scenarioProbabilities {0.2, 0.3, 0.5};
        std::vector<double> firstStageWeights {0.5, 1., 10., 10., 1., 1., 6., 8.5, 0.6, 1.2, 1.1, 15., 2., 0.1, 0.2};
        std::vector<std::vector<double>> secondStageWeights {{{2.5, 1.8, 10.5, 1.2, 0.05, 12.2, 6., 8.5, 1.6, 3.2, 0.1, 15., 2.25, 0.6, 0.1}, {19., 90., 5., 18., 20., 12.73, 3.14, 8.5, 9.6, 3.1, 1.1, 3.7, 2., 0.6, 2.2}, {0.5, 1., 3., 10., 1.6, 1., 0.6, 0.5, 14.6, 1.2, 3.2, 1.3, 0.02, 0.1, 5.2}}};

        FullyConnectedTwoStageMST mst(6, firstStageWeights, secondStageWeights, scenarioProbabilities);

        double res = solve_relaxed_lp(mst);
        mst.save_lp_result_map("lp_erste_simulation_approx");

        mst.approximate(rng);
        mst.save_approx_result_map("approx_erste_simulation_approx");
    }
    catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } 
    catch(...) {
        cout << "Exception during optimization" << endl;
    }
}