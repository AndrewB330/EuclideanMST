#pragma once
#include <algorithm>
#include <memory>
#include <cmath>

/**
 * Point in DIM-dimensional space
 */
template<size_t DIM>
class Point {

public:
    Point();

    const double & operator[](size_t i) const;
    double & operator[](size_t i);

    /**
     * Distance between two points
     */
    template<size_t DIM>
    friend double distance(const Point<DIM> & a, const Point<DIM> & b);

private:
    double data[DIM];
};

/**
 * Point with id
 */
template<size_t DIM>
class UPoint : public Point<DIM> {
public:

    UPoint(const Point<DIM> & ref, size_t id = 0);
    UPoint(size_t id = 0);

    size_t get_id() const;

private:
    size_t id;
};

/**
 * Bounding box in DIM-dimensional space
 */
template<size_t DIM>
class AABB {
public:
    AABB();
    /**
     * Construct bounding box from vector iterator
     */
    AABB(typename std::vector<UPoint<DIM>>::const_iterator iter, typename std::vector<UPoint<DIM>>::const_iterator end);

    /**
     * Size of the k-th dimension
     */
    double size(size_t k) const;

    /**
     * Get the index of a dimension with the largest size
     */
    size_t get_largest_dim() const;

    /**
     * Distance between two bounding boxes
     */
    template<size_t DIM>
    friend double distance(const AABB<DIM> & a, const AABB<DIM> & b);

private:
    Point<DIM> minimal;
    Point<DIM> maximal;
};

// Implementation

template<size_t DIM>
Point<DIM>::Point() {
    std::fill(data, data + DIM, 0.0);
}

template<size_t DIM>
const double & Point<DIM>::operator[](size_t i) const {
    return data[i];
}

template<size_t DIM>
double &  Point<DIM>::operator[](size_t i) {
    return data[i];
}

template<size_t DIM>
UPoint<DIM>::UPoint(const Point<DIM> & ref, size_t id) : Point<DIM>(ref), id(id) {}

template<size_t DIM>
UPoint<DIM>::UPoint(size_t id) :id(id) {}

template<size_t DIM>
size_t UPoint<DIM>::get_id() const {
    return id;
}

template<size_t DIM>
AABB<DIM>::AABB() {}

template<size_t DIM>
AABB<DIM>::AABB(typename std::vector<UPoint<DIM>>::const_iterator iter, typename std::vector<UPoint<DIM>>::const_iterator end) {
    minimal = maximal = *iter;
    for (; iter != end; ++iter) {
        for (size_t k = 0; k < DIM; k++) {
            minimal[k] = std::min(minimal[k], (*iter)[k]);
            maximal[k] = std::max(maximal[k], (*iter)[k]);
        }
    }
}

template<size_t DIM>
double AABB<DIM>::size(size_t k) const {
    return maximal[k] - minimal[k];
}

template<size_t DIM>
size_t AABB<DIM>::get_largest_dim() const {
    size_t biggest_dim = 0;
    for (size_t k = 1; k < DIM; k++) {
        if (size(k) > size(biggest_dim))
            biggest_dim = k;
    }
    return biggest_dim;
}

template<size_t DIM>
double distance(const Point<DIM> & a, const Point<DIM> & b) {
    double distance_sqr = 0;
    for (size_t k = 0; k < DIM; k++) {
        double diff = a[k] - b[k];
        distance_sqr += diff * diff;
    }
    return sqrt(distance_sqr);
}

template<size_t DIM>
double distance(const AABB<DIM> & a, const AABB<DIM> & b) {
    double distance_sqr = 0;
    for (size_t k = 0; k < DIM; k++) {
        double diff = std::max(a.minimal[k], b.minimal[k]) - std::min(a.maximal[k], b.maximal[k]);
        if (diff > 0) {
            distance_sqr += diff * diff;
        }
    }
    return sqrt(distance_sqr);
}