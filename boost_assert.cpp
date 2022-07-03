# include <iostream>
# include <assert.h>
// #include <cassert>

#include <boost/filesystem.hpp>

int main() {

    assert(2+2==4);

    boost::filesystem::path p("gss");
    p /= "Hi";

    std::cout << p.string() << std::endl;

    std::cout << "Ich bin der Boost assert Test\n";
}