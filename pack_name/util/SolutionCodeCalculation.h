#ifndef PSOMAB_PACK_NAME_UTIL_SOLUTIONCODECALCULATION_H_
#define PSOMAB_PACK_NAME_UTIL_SOLUTIONCODECALCULATION_H_

#include <Eigen/Core>

// calculate hash for eigen vector
namespace std {
template<>
struct hash<Eigen::VectorXi> {
        std::size_t operator()(const Eigen::VectorXi &k) const {
                std::size_t h = 0;
                for (int i = 0; i < k.size(); i++) {
                        h ^= std::hash<int>()(k[i]) + 0x9e3779b9 + (h << 6) + (h >> 2);
                }
                return h;
        }
};
}

#endif//PSOMAB_PACK_NAME_UTIL_SOLUTIONCODECALCULATION_H_
