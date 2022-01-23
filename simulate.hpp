#pragma once

// #include "two_stage_gurobi.hpp"
#include "two_stage.hpp"

enum class Vergleich {ApproxVs4b, ApproxVsTriv, ApproxVsBruteforce};

template <typename... Tn>
void simulate(int i, Tn..., double d);


// vllt ist ensemble auch falsche Bezeichnung
class Ensemble {

protected:
    unsigned int number_nodes;
    TwoStageProblem two_stage_problem;

    virtual void erase_all_edges();
    // virtual void add_edges() = 0;        Das mache ich in allen Childklassen individuell

public:

    // will ich nen default constructor?

    virtual ~Ensemble() = default;

    // constructor
    Ensemble(unsigned int number_nodes);

    // soll das Ensemble neu aufsetzen, zB. einen neuen Baum erzeugen und neue Kosten
    // virtual void recreate() = 0;        // mache ich in den childklassen individuell

    double approx();
    double bruteforce();
    double do4b();
    double greedy();

    // speichert mir den aktuellen graphen des twostageproblems
    virtual void save_current_graph(std::string name);

};


class NewEdgeCostCreator {

    private:
        // RNG & rng;
        // std::uniform_real_distribution<double> uniformDist;

    public:
        virtual ~NewEdgeCostCreator() = default;

};

class Tree : public Ensemble {

public:
    Tree(unsigned int number_nodes, std::mt19937 & rng);

    virtual ~Tree() = default;
    // virtual void recreate() override;               // was passiert mit dieser methode???, wird die versteckt?
    virtual void recreate(std::mt19937 & rng);

    virtual void add_edges(std::mt19937 & rng);

};

class FullyConnected : public Ensemble {

    public:
    FullyConnected(unsigned int number_nodes);
    virtual ~FullyConnected() = default;
    virtual void recreate();

};

void simulate(unsigned int runs, Ensemble & ensemble, Vergleich vergleich);
