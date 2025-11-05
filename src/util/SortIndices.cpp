#include "SortIndices.h"

bool sort_indices_comp(const std::pair<int, double> &left, const std::pair<int, double> &right) {
        return left.second < right.second;
}

std::vector<int> sort_indices(const Eigen::VectorXd &x) {
        std::vector<int> indices;
        std::vector<std::pair<int, double>> data(x.size());
        for (int i = 0; i < x.size(); i++) {
                data[i].first = i;
                data[i].second = x(i);
        }
        std::sort(data.begin(), data.end(), sort_indices_comp);
        for (auto &i : data) {
                indices.push_back(i.first);
        }
        return indices;
}
