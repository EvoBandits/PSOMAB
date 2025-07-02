#include "../src/util/SortIndices.h"
#include "Eigen/Core"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("sort_indices sorts vector indices correctly", "[sort_indices]") {
        // Create an unsorted vector
        Eigen::VectorXd unsorted_vector(5);
        unsorted_vector << 5.0, 2.0, 1.0, 4.0, 3.0;

        // Call the sort_indices function to get the sorted indices
        std::vector<int> sorted_indices = sort_indices(unsorted_vector);

        // Check that the indices are sorted correctly
        REQUIRE(sorted_indices[0] == 2);
        REQUIRE(sorted_indices[1] == 1);
        REQUIRE(sorted_indices[2] == 4);
        REQUIRE(sorted_indices[3] == 3);
        REQUIRE(sorted_indices[4] == 0);
}