# include <random>
# include <iostream>

int main() {

    std::mt19937 rng{1000};     // 1000er seed
    std::uniform_real_distribution<double> dist(0, 10);


    for (int i=0; i<10; i++) {
        std::cout << dist(rng) << std::endl;
    }

    std::cout << "Seed reset\n";

    rng.seed(1000);     // 1000er seed

    for (int i=0; i<10; i++) {
        std::cout << dist(rng) << std::endl;
    }
}