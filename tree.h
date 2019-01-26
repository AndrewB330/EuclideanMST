#pragma once
#include <algorithm>
#include <vector>
#include <limits>
#include <memory>
#include "model.h"

/**
 * K-d tree data structure
 */
template<size_t DIM>
struct KdTree {
public:
    KdTree();
    /**
     * Construct K-d tree with height at most max_height from a given set of points
     */
    KdTree(const std::vector<Point<DIM>> & points, size_t max_height);

    size_t get_root_id() const;
    size_t get_maximal_id() const;
    size_t get_left_child_id(size_t id) const;
    size_t get_right_child_id(size_t id) const;
    const AABB<DIM> & get_bounding_box(size_t id) const;
    bool is_leaf(size_t id) const;

    typename std::vector<UPoint<DIM>>::const_iterator points_begin(size_t node_id) const;
    typename std::vector<UPoint<DIM>>::const_iterator points_end(size_t node_id) const;

private:
    /**
     * K-d tree node. Represents part of space with points from (start) to (start+size-1) position in points array
     */
    struct KdNode {
        KdNode() : start(0), is_leaf(false), size(0) {}
        AABB<DIM> aabb;
        bool is_leaf; //is this node a leaf node and has no childs
        size_t start;
        size_t size;
    };
    /**
     * Recursively build tree for current node node_id, for points from position (start) to position (start+size-1)
     */
    void build(size_t node_id, size_t start, size_t size, size_t remaining_height);

    std::vector<UPoint<DIM>> points;
    std::vector<KdNode> nodes;
};

// Implementation

template<size_t DIM>
KdTree<DIM>::KdTree() {}

template<size_t DIM>
KdTree<DIM>::KdTree(const std::vector<Point<DIM>> & points, size_t max_height) {
    this->points.reserve(points.size());
    for (auto & p : points) {
        this->points.push_back(UPoint<DIM>(p, this->points.size()));
    }
    nodes.resize(points.size() * 4);
    build(1, 0, points.size(), max_height);
}

template<size_t DIM>
size_t KdTree<DIM>::get_root_id() const {
    return 1;
}

template<size_t DIM>
size_t KdTree<DIM>::get_maximal_id() const {
    return (nodes.empty() ? 0 : nodes.size() - 1);
}

template<size_t DIM>
size_t KdTree<DIM>::get_left_child_id(size_t id) const {
    return id * 2;
}

template<size_t DIM>
size_t KdTree<DIM>::get_right_child_id(size_t id) const {
    return id * 2 + 1;
}


template<size_t DIM>
bool KdTree<DIM>::is_leaf(size_t id) const {
    return nodes[id].is_leaf;
}

template<size_t DIM>
const AABB<DIM> & KdTree<DIM>::get_bounding_box(size_t id) const {
    return nodes[id].aabb;
}

template<size_t DIM>
typename std::vector<UPoint<DIM>>::const_iterator KdTree<DIM>::points_begin(size_t node_id) const {
    return points.begin() + nodes[node_id].start;
}

template<size_t DIM>
typename std::vector<UPoint<DIM>>::const_iterator KdTree<DIM>::points_end(size_t node_id) const {
    return points.begin() + nodes[node_id].start + nodes[node_id].size;
}

template<size_t DIM>
void KdTree<DIM>::build(size_t node_id, size_t start, size_t size, size_t remaining_height) {
    nodes[node_id].aabb = AABB<DIM>(points.cbegin() + start, points.cbegin() + start + size);
    nodes[node_id].start = start;
    nodes[node_id].size = size;

    if (size == 1 || remaining_height == 0) {
        nodes[node_id].is_leaf = true;
        return;
    }

    size_t biggest = nodes[node_id].aabb.get_largest_dim();

    std::nth_element(points.begin() + start, points.begin() + start + size / 2, points.begin() + start + size,
        [&biggest](const Point<DIM> & a, const Point<DIM> & b) {
        return a[biggest] < b[biggest];
    });

    // recursively call build for two parts
    build(node_id * 2, start, size / 2, remaining_height - 1);
    build(node_id * 2 + 1, start + size / 2, (size + 1) / 2, remaining_height - 1);
}