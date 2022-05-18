#include <random>
#include <ctime>
#include <iostream>

int main() {

    std::mt19937 rng{1000};

    std::uniform_int_distribution<int> dist(1, 99);

    for (int i=0; i<10; i++) {
        std::cout << dist(rng) << "\n";
    }
}