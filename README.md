# MEMO PSO

MEMO PSO is a C++ library for solving stochastic optimization problems using a memory-enhanced particle swarm optimizer.

[![](https://github.com/H3nkl3r/PSOMAB/actions/workflows/cmake.yml/badge.svg)](https://github.com/H3nkl3r/PSOMAB/actions/workflows/cmake.yml)

## Installation

To install the library, clone the repository and initialize submodules:

```bash
git clone 
git submodule update --init --recursive
```

## Usage

To use MEMO PSO, include the relevant headers and create an instance of the MEMOPSO class:

```c++
#include "./pack_name/algo/particle_swarm_optimizer_multi_armed_bandit/PSOMAB.h"
#include "./problems/ackley/ackley.h"

#include <iostream>

PSOMAB psomab_instance = PSOMAB(ackley, 10000, 10, ackley_lb, ackley_ub, ackley_dim, false);
psomab_instance.optimize();

std::cout << psomab_instance.get_best_solution().back().get_action_vector() << std::endl;
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Licence

MEMO PSO is open-source software released under the [AGPLv3]() license. This means that any derivative work based on the publicly available source code also needs to be licensed under AGPLv3. However, we can provide a different license for closed-source use cases.

## Credit

MEMO PSO was developed by:

* Deniz Preil
* Timo Kühne
* Jonathan Laib

## Citing memo-pso

If you use MEMO PSO in your scientific work, please cite it as follows:

tbd

## Third Party Dependencies
MEMO PSO relies on external libraries to provide its features. You can find these libraries in the /third_party folder.
