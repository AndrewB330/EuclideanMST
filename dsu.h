#pragma once
#include <vector>

/**
 * Disjoint-set-union data structure
 */
class DSU {

public:

    DSU(size_t n = 0);

    size_t get_set(size_t x) const;
    bool is_in_same_set(size_t x, size_t y) const;

    void reset(size_t n);
    bool unite(size_t x, size_t y);

private:

    mutable std::vector<size_t> p;
    std::vector<size_t> rank;
};

// Implementation

DSU::DSU(size_t n) {
    reset(n);
}


size_t DSU::get_set(size_t x) const {
    return p[x] == x ? x : p[x] = get_set(p[x]);
}

bool DSU::is_in_same_set(size_t x, size_t y) const {
    return get_set(x) == get_set(y);
}

void DSU::reset(size_t n) {
    p.resize(n);
    rank.assign(n, 0);
    for (size_t i = 0; i < n; i++) {
        p[i] = i;
    }
}

bool DSU::unite(size_t x, size_t y) {
    size_t set_a = get_set(x);
    size_t set_b = get_set(y);
    if (set_a == set_b) {
        return false;
    }
    if (rank[set_a] > rank[set_b]) {
        std::swap(set_a, set_b);
    }
    if (rank[set_a] == rank[set_b]) {
        rank[set_b]++;
    }
    p[set_a] = set_b;
    return true;
}