#include "utilities.hpp"
// #include <ctime>

// #include <iostream>

// define the RNG
std::mt19937 rng{std::random_device{}()};               // construct it with the seed that I get from the random_device rvalue on which I use the ()-operator (functor)


std::vector<double> calcScenarioProbabilities(size_t n, std::mt19937 & rng) {

    // const auto seed = time(nullptr);
    // std::mt19937 eng(static_cast<std::mt19937::result_type>(seed));

    std::uniform_real_distribution<double> dist(1, 1000);                // Intervall anders machen? Dazu gibts schriftliche Ueberlegungen vom 15.04.21

    double sum = 0;
    std::vector<double> vec;
    vec.reserve(n);
    for (int i=0; i<n; i++) {
        double res = dist(rng);
        vec.push_back(res);
        sum += res;
    }

    for (int i=0; i<n; i++) {
        vec[i] /= sum;
    }

    return vec;
} 

// int main() {

//     double n = 5;
//     auto res = calcScenarioProbabilities(n);
//     for (int i=0; i<n; i++) {
//         std::cout << res[i] << ", ";
//     }
//     std::cout << "\n";
// }

void inittwiddle(int m, int n, std::vector<int> & p) {
  int i;
  p[0] = n+1;
  for(i = 1; i != n-m+1; i++)
    p[i] = 0;
  while(i != n+1)
    {
    p[i] = i+m-n;
    i++;
    }
  p[n+1] = -2;
  if(m == 0)
    p[1] = 1;
  }

bool twiddle(int & x, int & y, int & z, std::vector<int> & p) {

    int i, j, k;
    j = 1;
    while(p[j] <= 0)
        j++;
    if(p[j-1] == 0) {
        for(i = j-1; i != 1; i--)
            p[i] = -1;
        p[j] = 0;
        x = z = 0;
        p[1] = 1;
        y = j-1;
    }
    else {
        if(j > 1)
            p[j-1] = 0;
        do
            j++;
        while(p[j] > 0);
        k = j-1;
        i = j;
        while(p[i] == 0)
            p[i++] = -1;
        if(p[i] == -1) {
            p[i] = p[k];                // lande ich hier auch bei p[k] = 2 oder noch groesser?
            z = p[k]-1;
            x = i-1;
            y = k-1;
            p[k] = -1;
        }
        else {
            if(i == p[0])
	            return(1);
            else {
	            p[j] = p[i];
	            z = p[i]-1;
	            p[i] = 0;
	            x = j-1;
	            y = i-1;
	        }
        }
    }
    return(0);
}