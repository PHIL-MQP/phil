#include <iostream>
#include <vector>
#include "csvReader.h"

int main(int argc, char **argv) {
    std::vector<std::vector<float>> data_array = getData(std::string(argv[1]), 1, 100);

    for (int j = 0; j < data_array.size(); j++) {
        for (int i = 0; i < data_array[j].size(); i++) {
            std::cout << data_array[j][i] << std::endl;
        }
    }
    return 0;
}
