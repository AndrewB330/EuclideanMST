#pragma once
#include "model.h"
#include "tree.h"
#include "dsu.h"

typedef std::pair<size_t, size_t> Edge;

template<size_t DIM>
class EmstSolver {
public:
    EmstSolver() {}

    const std::vector<Edge> & get_solution() const { return solution; }
    const double & get_total_length() const { return total_length; }

protected:
    std::vector<Edge> solution;
    double total_length = 0.0;
};

/**
 * Implementation of EMST algorithm using K-d tree
 * "Fast Euclidean Minimum Spanning Tree: Algorithm, Analysis, and Applications. William B. March, Parikshit Ram, Alexander G. Gray"
 * Time complexity: O(cNlogN), extra constant c depends on the distribution of points
 */
template<size_t DIM>
class KdTreeSolver : public EmstSolver<DIM> {
public:
    KdTreeSolver(std::vector<Point<DIM>> & points) :num_points(points.size()) {
        dsu.reset(num_points);
        tree = KdTree<DIM>(points, floor(log2(num_points)) - 2);
        is_fully_connected.assign(tree.get_maximal_id() + 1, false);
        solve();
        // todo: clear containers
    }

private:
    void solve() {
        auto & solution = EmstSolver<DIM>::solution;
        auto & total_length = EmstSolver<DIM>::total_length;

        while (solution.size() + 1 < num_points) {
            node_approximation.assign(tree.get_maximal_id() + 1, std::numeric_limits<double>::max());
            nearest_set.assign(num_points, { std::numeric_limits<double>::max(), Edge(0,0) });

            check_fully_connected(tree.get_root_id());

            find_component_neighbors(tree.get_root_id(), tree.get_root_id());

            for (size_t i = 0; i < num_points; i++) {
                if (i == dsu.get_set(i)) {
                    Edge e = nearest_set[i].second;
                    if (dsu.unite(e.first, e.second)) {
                        solution.push_back(e);
                        total_length += nearest_set[i].first;
                    }
                }
            }
        }
    }

    void find_component_neighbors(size_t q, size_t r, size_t depth = 0) {
        if (is_fully_connected[q] && is_fully_connected[r] &&
            dsu.is_in_same_set(tree.points_begin(q)->get_id(), tree.points_begin(r)->get_id())) {
            return;
        }
        if (distance(tree.get_bounding_box(q), tree.get_bounding_box(r)) > node_approximation[q]) {
            return;
        }
        if (tree.is_leaf(q) && tree.is_leaf(r)) {
            node_approximation[q] = 0.0;
            for (auto i = tree.points_begin(q); i != tree.points_end(q); i++) {
                for (auto j = tree.points_begin(r); j != tree.points_end(r); j++) {
                    if (!dsu.is_in_same_set(i->get_id(), j->get_id())) {
                        double dist = distance(*i, *j);
                        if (dist < nearest_set[dsu.get_set(i->get_id())].first) {
                            nearest_set[dsu.get_set(i->get_id())] = { dist, { i->get_id(), j->get_id() } };
                        }
                    }
                }
                node_approximation[q] = std::max(node_approximation[q], nearest_set[dsu.get_set(i->get_id())].first);
            }
        } else {
            size_t qleft = tree.get_left_child_id(q);
            size_t qright = tree.get_right_child_id(q);
            size_t rleft = tree.get_left_child_id(r);
            size_t rright = tree.get_right_child_id(r);
            if (tree.is_leaf(q)) {
                find_component_neighbors(q, rleft, depth);
                find_component_neighbors(q, rright, depth);
                return;
            }
            if (tree.is_leaf(r)) {
                find_component_neighbors(qleft, r, depth);
                find_component_neighbors(qright, r, depth);
                node_approximation[q] = std::max(node_approximation[qleft], node_approximation[qright]);
                return;
            }
            find_component_neighbors(qleft, rleft, depth + 1);
            find_component_neighbors(qleft, rright, depth + 1);
            find_component_neighbors(qright, rright, depth + 1);
            find_component_neighbors(qright, rleft, depth + 1);
            node_approximation[q] = std::max(node_approximation[qleft], node_approximation[qright]);
        }
    }

    void check_fully_connected(size_t node_id) {
        if (is_fully_connected[node_id]) {
            return;
        }
        if (tree.is_leaf(node_id)) {
            bool fully_connected = true;
            for (auto iter = tree.points_begin(node_id); iter + 1 != tree.points_end(node_id); ++iter) {
                fully_connected &= dsu.is_in_same_set(iter->get_id(), (iter + 1)->get_id());
            }
            is_fully_connected[node_id] = fully_connected;
            return;
        }
        size_t left = tree.get_left_child_id(node_id);
        size_t right = tree.get_right_child_id(node_id);
        check_fully_connected(left);
        check_fully_connected(right);
        if (is_fully_connected[left] && is_fully_connected[right] &&
            dsu.is_in_same_set(tree.points_begin(left)->get_id(), tree.points_begin(right)->get_id())) {
            is_fully_connected[node_id] = true;
        }
    }

    size_t num_points;
    DSU dsu;
    KdTree<DIM> tree;
    std::vector<bool> is_fully_connected;
    std::vector<double> node_approximation;
    std::vector<std::pair<double, Edge>> nearest_set;
};

/**
 * Prim's algorithm
 * Time complexity: O(N^2)
 */
template<size_t DIM>
class PrimSolver : public EmstSolver<DIM> {
public:
    PrimSolver(std::vector<Point<DIM>> & points) {
        solve(points);
    }

private:
    void solve(const std::vector<Point<DIM>> & points) {
        auto & solution = EmstSolver<DIM>::solution;
        auto & total_length = EmstSolver<DIM>::total_length;
        size_t num_points = points.size();

        std::vector<std::pair<double, size_t>> distance_to_tree(num_points, { std::numeric_limits<double>::max(), 0 });
        std::vector<bool> used(num_points, false);
        used[0] = true;
        for (size_t i = 1; i < num_points; i++) {
            distance_to_tree[i] = { distance(points[0], points[i]), 0 };
        }

        for (size_t iteration = 1; iteration < num_points; iteration++) {
            size_t nearest_id = 0;
            for (size_t i = 1; i < num_points; i++) {
                if (!used[i] && distance_to_tree[i] < distance_to_tree[nearest_id]) {
                    nearest_id = i;
                }
            }
            solution.push_back({ nearest_id, distance_to_tree[nearest_id].second });
            total_length += distance_to_tree[nearest_id].first;
            used[nearest_id] = true;
            for (size_t i = 1; i < num_points; i++) {
                if (!used[i] && distance(points[i], points[nearest_id]) < distance_to_tree[i].first) {
                    distance_to_tree[i] = { distance(points[i], points[nearest_id]) , nearest_id };
                }
            }
        }
    }

};