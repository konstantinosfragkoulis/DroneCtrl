#include <cstdio>
#include <getopt.h>

#include "DroneCtrl.hpp"

using namespace DroneCtrl;

void printVersion() {
    printf("DroneCtrl version %s\n", VERSION.c_str());
}

void printHelp() {
    printf("Usage: DroneCtrl [options]\n");
    printf("Options:\n");
    printf("  -v, --version   Show version information\n");
    printf("  -h, --help      Show this help message\n");
}

void parseArguments(int argc, char *argv[]) {
    const char* const short_opts = "vh";
    const option long_opts[] = {
        {"version", no_argument, nullptr, 'v'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, no_argument, nullptr, 0}
    };

    while (true) {
        const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);

        if (opt == -1) {
            break;
        }

        switch (opt) {
            case 'v':
                printVersion();
                exit(0);
            case 'h':
                printHelp();
                exit(0);
            case '?': // Unrecognized option
            default:
                printHelp();
                exit(1);
        }
    }
}