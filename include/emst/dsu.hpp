#pragma once
#include <vector>

/**
 * Disjoint-set-union data structure
 */
class DSU {

public:

    explicit DSU(size_t n = 0);

    size_t get_set(size_t x) const;
    bool is_in_same_set(size_t x, size_t y) const;

    void reset(size_t n);
    bool unite(size_t x, size_t y);

private:

    mutable std::vector<size_t> p;
    std::vector<size_t> rank;
};
