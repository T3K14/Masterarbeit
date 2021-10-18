#include "two_stage_gurobi.hpp"
#include "two_stage.hpp"
#include <lemon/hao_orlin.h>
#include <algorithm>

void solve_relaxed_lp(TwoStageProblem & two_stage_problem, lemon::ListGraph::EdgeMap<std::vector<double>> & result_optimized_values_map) {

    GRBEnv env = GRBEnv(true);

    GRBModel model = GRBModel(env);

    lemon::ListGraph::EdgeMap<GRBVar *> gurobi_variables_map(two_stage_problem.g);

    // das wird die objective function 
    GRBLinExpr obj = 0.0;

    // ich tue so, als haette ich die edges nicht zwangsweise selbst in einem array sondern nutze den lemon edge iterator
    for (lemon::ListGraph::EdgeIt e(two_stage_problem.g); e != lemon::INVALID; ++e) {
        
        // jede Kante bekommt array mit Variablen fuer alle Szenarios (+1 fuer die erste Stage)
        gurobi_variables_map[e] = model.addVars(numberScenarios + 1, GRB_CONTINUOUS);               // werden unten gefreet

        // und ich kann gleich schon die objektive function mit aufbauen
        // stage 1
        obj += firstStageCosts[e] * gurobi_variables_map[e][0];

        // stage 2
        for (int i=1; i<numberScenarios+1; i++) {
            obj += scenarioProbabilities[i] * secondStageCosts[e][i] * gurobi_variables_map[e][i];
        }
    }

    model.setObjective(obj, GRB_MINIMIZE);

    // jetzt so lange Cut-Constraints hinzufuegen, bis die Bedingungen immer erfuellt sind 
    while(true) {

        model.optimize();

        // map der capacities (wird fuer jedes szenario neu beschrieben), brauche ich fuer den HaoOrlin-Algorithmus
        ListGraph::EdgeMap<double> capacity_map(g);

        // map in der angegeben wird, welche Knoten in dereinen MinCut-Teilmenge drin sind, output des HaoOrlin-Algorithmus, aus dem ich dann die Kanten bestimmen kann, die die Cut-
        // Teilmengen verbinden
        ListGraph::NodeMap<bool> min_cut_result_map(g);

        // gehe alle szenarien durch und suche nach mincut, der die Bedingung nicht erfuellt
        for (int i=1; i<numberScenarios+1; i++) {

            // die x^_e und x^i_e Werte addieren und als capacity Werte eintragen
            for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
                // die capacity ist die summe aus den Werten der ersten Phase und den der i-ten Phase
                //capacity_map[edges[j]] = variables_array[j].get(GRB_DoubleAttr) + varables_array[i * numberEdges + j].get(GRB_DoubleAttr);
                capacity_map[e] = gurobi_variables_map[e][0].get(GRB_DoubleAttr) + gurobi_variables_map[e][i].get(GRB_DoubleAttr);
            }

            // suche minCut
            HaoOrlin hao(g, capacity_map);
            hao.init();
            hao.calculateIn();

            // falls der minCut die Bedingung verletzt, der Aufruf speichert direkt auch die bools fuer die Teilmengen in min_cut_result_map
            if(hao.minCutMap(min_cut_result_map) < 1) {
                
                // fuege neues constraint hinzu, damit diese Bedingung in zukunft erfuellt ist
                GRBLinExpr constraint = 0.0;

                // dazu muss ich erstmal alle Kanten ermitteln, die bei diesem minCut die beiden Teilmengen verbinden
                for (lemon::ListGraph::NodeIt n(g); n != lemon::INVALID; ++n) {
                    if (min_cut_result_map[n]) {
                        for (lemon::ListGraph::IncEdgeIt e(g, n); e != lemon::INVALID; ++e) {
                            // wenn entweder n true und die andere edge false ist oder andersrum
                            if ((g.source(e) && !g.target(e)) || (!g.source(e) && g.target(e))) {

                                constraint += gurobi_variables_map[e][0] + gurobi_variables_map[e][i];

                            }
                        }
                    }
                }

                // constraint jetzt noch hinzufuegen
                model.addConstr(constraint);

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
    }


    // ich schreibe nun in die uebergeben EdgeMap die Ergebnisse der optimierten LP-Variablen und free die hier allocateten Variablen arrays
    for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {

        std::copy_n(model.get(GRB_Double_Attr, gurobi_variables_map[e], numberScenarios+1), result_optimized_values_map[e].size(), result_optimized_values_map[e].begin());

        // hier free ich die Variablen arrays pro Kante
        delete[] gurobi_variables_map[e];

    }

    // for (lemon::ListGraph::EdgeIt e(g); e != lemon::INVALID; ++e) {
        // delete[] gurobi_variables_map[e];
    // }
    

    //vielleicht brauche ich auch gar nicht die Variablen zurueckgeben, sondern nur deren Werte

    // return gurobi_variables_map;
}


