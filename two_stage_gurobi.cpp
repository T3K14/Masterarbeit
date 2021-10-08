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

    // GRBVar variables_array[][]

    GRBLinExpr obj = 0.0;

    // die ersten Summanden fuer die first stage variablen zur objective hinzufuegen
    for (int i=0; i < numberEdges, i++) {
        obj += firstStageCosts[edges[i]] * variables_array[i];
    }

    // die uebrigen Summanden der anderen Second stage scenarien hinzufuegen3
    for (int i=numberEdges; i < number_coefficients; i++) {

        // aktuelle Kante und aktuelles Szenario
        unsigned int edge_num = i % numEdges;
        unsigned int secnario_num = (i - edge_num) % numberScenarios;
        obj += scenarioProbabilities[secnario_num] * secondStageCosts[edges[edge_num]][secnario_num] * variables_array[i]
    }
    model.setObjective(obj, GRB_MINIMIZE);

    while(true) {

        model.optimize();

        // map der capacities (wird fuer jedes szenario neu beschrieben)
        ListGraph::EdgeMap<double> capacity_map(g);

        // map in der angegeben wird, welche Knoten in dereinen MinCut-Teilmenge drin sind
        ListGraph::NodeMap<bool> min_cut_result_map(g);

        // gehe alle szenarien durch und suche nach mincut, der die Bedingung nicht erfuellt
        for (int i=0; i<numberScenarios; i++) {
            


            // die aktuellen Werte der Variablen holen
            // double * lp_ergebnisse = model.get(GRB_DoubleAttr, scenario_variables.data());



            // die x^_e und x^i_e Werte addieren und als capacity Werte eintragen
            for (int j=0; j<numberEdges; j++) {
                // die capacity ist die summe aus den Werten der ersten Phase und den der i-ten Phase
                capacity_map[edges[j]] = variables_array[j].get(GRB_DoubleAttr) + varables_array[i * numberEdges + j].get(GRB_DoubleAttr);
            }


            HaoOrlin hao(g, capacity_map);
            hao.init();
            hao.calculateIn();




            // falls der minCut die Bedingung verletzt 
            if(hao.minCutMap(min_cut_result_map) < 1) {
                
                // fuege neues constraint hinzu, damit diese Bedingung in zukunft erfuellt ist
                


                // und ich gehe aus derm for-loop raus, in der Annahme, dass allein diese Veraenderung schon was bewirkt und es sich vielleicht nicht lohnt, noch weiter durch alle anderen
                //Szenarien zu schauen
                break;
            } 

        }

        // falls an diesem Punkt der minCut das Constraint erfuelt, gibt es kein Szenario mehr, wo der minCut gegen das Constraint verstoest und ich bin fertig mit der LP-Loesung
        if(hao.minCutValue() >= 1) {
            break;
        }
        
        // ansonsten optimiere erneut


        // delete[] lp_ergebnisse;

    }

    delete[] variables_array;
}