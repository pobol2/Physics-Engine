#include "common.hpp"

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
    try {
        const auto options = physics::app::parseOptions(argc, argv);
        if (options.help) { std::cout << physics::app::usage(argv[0]); return 0; }
        if (options.list) { std::cout << physics::app::exampleList(); return 0; }
        physics::app::Run run(options);
        while (!run.finished()) run.advance();
        run.finishOutput();
        std::cout << run.summary() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
