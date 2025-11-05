# PSOMAB

PSOMAB is a C++ library for solving stochastic optimization problems using multi armed bandits.

[![](https://github.com/EvoBandits/PSOMAB/actions/workflows/tests.yml/badge.svg)](https://github.com/EvoBandits/PSOMAB/actions/workflows/tests.yml)

## Installation

To install the library, clone the repository and initialize submodules:

```bash
git clone https://github.com/EvoBandits/PSOMAB.git
git submodule update --init --recursive
```

To build and run PSOMAB, you need to have CMake and Ninja installed. You can install them using the following commands:

```bash
sudo apt install cmake ninja-build
```
Then, create a build directory and generate the project build system:

```bash
mkdir build
cmake -DCMAKE_MAKE_PROGRAM=ninja -G Ninja -S . -B ./build
```

Finally, build the project and run PSOMAB (make sure to add a configuration file):
```bash
cmake --build ./build --target PSOMAB -j 6
./build/PSOMAB
```

## Usage

To use PSOMAB, include the relevant headers and create an instance of the PSOMAB class:

```c++
#include "./src/algo/particle_swarm_optimizer/variants/memory_enhanced/PSOMAB.h"

#include <iostream>

double problem_function(const std::vector<double> &x) {
    double a = 20;
    double b = 0.2;

    double sum_sq = x.array().square().sum() / 100.0;
    double sum_cos = (x.array().cos()).sum();

    return -a * exp(-b * sqrt(sum_sq / ackley_dim)) - exp(sum_cos / ackley_dim) + a + exp(1);
}

int main() {
    int function_dim = 2;
    Eigen::Vector2i function_lb(-500, -500);
    Eigen::Vector2i function_ub(500, 500);
    int max_function_evaluations = 10000;

    PSOMAB psomab_instance = PSOMAB(problem_function, function_lb, function_ub, function_dim, max_function_evaluations);
    psomab_instance.optimize();

    std::cout << psomab_instance.best_solution().back().get_action_vector() << std::endl;
    return 0;
}
```
For a complete example, please refer to the `tests/main.cpp` file in the repository.

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Licence

PSOMAB is open-source software released under the [AGPLv3](https://github.com/EvoBandits/PSOMAB?tab=AGPL-3.0-1-ov-file) license. This means that any derivative work based on the publicly available source code also needs to be licensed under AGPLv3. However, we can provide a different license for closed-source use cases.

## Credit

PSOMAB was developed by:

* Deniz Preil
* Timo Kühne
* Jonathan Laib

## Citing PSOMAB

If you use PSOMAB in your scientific work, please cite it as follows:

tbd

## Third Party Dependencies
PSOMAB relies on external libraries to provide its features. You can find these libraries in the /third_party folder.
