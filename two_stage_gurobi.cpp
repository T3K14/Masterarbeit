#include "two_stage_gurobi.hpp"
#include "gurobi_c++.hpp"
#include <lemon/hao_orlin.h>


void approximate() {

    GRBEnv env = GRBEnv(true);

    GRBModel model = GRBModel(env);

    //std::array<GRBVar, numberScenarios> variables;

    // lower bound (if NULL -> 0), upper bound (if Null -> infty), objective coefficients (null -> we set the coefficients later)
    unsigned int number_coefficients = numberEdges * (numberScenarios + 1) 
    GRBVar * variables_array = model.addVars(number_coefficients, GRB_CONTINUOUS);      // die anordnung, wenn ich zB. 4 szenarios und 3 kanten habe: x^1_e1, x^1_e2, x^1_e3, x^2_e1, x^2_e2, x^2_e3, x^3_e1, ...

    GRBLinExpr obj = 0.0;

    // die ersten Summanden fuer die first stage variablen zur objective hinzufuegen
    for (int i=0; i < numberEdges, i++) {
        obj += firstStageCosts[edges[i]] * variables_array[i];
    }

    // die uebrigen Summanden der anderen Second stage scenarien hinzufuegen3
    for (int i=numberEdges; i < number_coefficients; i++) {

        unsigned int edge_num = i % numEdges;
        unsigned int secnario_num = (i - edge_num) % numberScenarios;
        obj += scenarioProbabilities[secnario_num] * secondStageCosts[edges[edge_num]][secnario_num] * variables_array[i]
    }
    model.setObjective(obj, GRB_MINIMIZE);

    while(true) {

        

        // gehe alle szenarien durch und suche nach mincut, der die Bedingung nicht erfuellt
        for (int i=0; i<numberScenarios; i++) {
            
            std::array<GRBVar, numberEdges> scenario_variables;

            // die aktuellen Werte der Variablen holen
            double * lp_ergebnisse = model.get(GRB_DoubleAttr, )


            ListGraph::EdgeMap<double> capacities(g);

            // die x^_e und x^i_e Werte addieren und als capacity Werte eintragen
            for (int j=0; j<numberEdges; j++) {
                capacities[edges[j]] = 
            }


            HaoOrlin hao(g, capacity_map);
            hao.init();
            hao.calculateIn();

        }

        // fuege neues constraint hinzu, damit diese Bedingung in zukunft erfuellt ist

        // optimiere erneut


        delete[] lp_ergebnisse;

    }

    delete[] variables_array;
}