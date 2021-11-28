#include "utilities.hpp"
#include <vector>

/*
hat erfolgreich getestet, dass mit dem neuen Schema zum durchprobieren aller 1. stage Auswahlen ohne Stopbedingung alle moeglichen Faelle abgedeckt werden
*/

using namespace std;

int main() {

    vector<int> c = {0};
    int counter = 1;        // zum testen, ob auch wirklich alle Faelle durchgegangen werden (startet bei 1, weil ich kann auch keine Kante in Stage 1 kaufen)
    // 6 Kanten
    while (!c.empty()) {
        counter++;
        for(int & i : c) {
            cout << i << ", ";
        }
        cout << "\n";

        update_c(c, 6, false);
    }
    cout << "Counter: " << counter << endl;

    int s = 0;
    for (int i=0; i<7; i++) {
        s += BinomialCoefficient(6, i);
    }

    cout << "Summe der Binomialkoeffizienten: " << s << "\n";

    //jetzt mit Stop bei 0,1,3 (dabei fallen 3 Faelle weg)

    cout << "\n\nJetzt mit stop-Bed.\n";

    counter = 1;
    vector<int> c2 = {0};
    // 6 Kanten
    while (!c2.empty()) {
        bool stop = false;
        counter++;
        for(int & i : c2) {
            cout << i << ", ";
        }
        cout << "\n";

        if (c2.size()==3) {
            if (c2[0]==0 && c2[1]==1 && c2[2]==3) {
                stop = true;
            }
        }

        update_c(c2, 6, stop);
    }
    cout << "Counter: " << counter << endl;


    return 0;
}