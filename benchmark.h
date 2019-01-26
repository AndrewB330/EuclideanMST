#pragma once
#include <iostream>
#include <iomanip>
#include <cassert>
#include <chrono>
#include "emst.h"
using namespace std::chrono;

double uniform() {
    return rand() * 1.0 / RAND_MAX;
}

template<size_t DIM>
std::vector<Point<DIM>> generate_points_uniform(size_t num_points, double MAX = 100.0) {
    std::vector<Point<DIM>> points;
    for (size_t i = 0; i < num_points; i++) {
        Point<DIM> p;
        for (size_t k = 0; k < DIM; k++) {
            p[k] = (uniform() * 2.0 - 1.0) * MAX;
        }
        points.push_back(p);
    }
    return points;
}

template<size_t DIM>
std::vector<Point<DIM>> generate_points_gauss(size_t num_points, double MAX = 100.0) {
    std::vector<Point<DIM>> points;
    for (size_t i = 0; i < num_points; i++) {
        Point<DIM> p;
        for (size_t k = 0; k < DIM; k++) {
            p[k] = sqrt(-2 * log(uniform())) * cos(2 * 3.141592 * uniform()) * MAX;
        }
        points.push_back(p);
    }
    return points;
}

struct BenchmarkResult {
    double answer;
    double time;
};

template<typename EmstSolverType, size_t DIM>
BenchmarkResult run_benchmark(const std::vector<Point<DIM>> & points, size_t samples = 1) {
    BenchmarkResult result;
    auto start = std::chrono::system_clock::now();
    for (size_t i = 0; i < samples; i++) {
        EmstSolverType solver(points);
        result.answer = solver.get_total_length();
    }
    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    result.time = elapsed_seconds.count() / samples;
    return result;
}

template<size_t DIM>
void run_becnhmarks(const std::vector<Point<DIM>> & points, size_t samples, std::ostream & cout, std::string distribution = "none") {
    auto result_kd = run_benchmark<KdTreeSolver<DIM>, DIM>(points, samples);
    auto result_prim = (points.size() < 100000 ? run_benchmark<PrimSolver<DIM>, DIM>(points, samples / 50 + 1) : BenchmarkResult{ result_kd.answer, 0 });

    assert(fabs(result_kd.answer / result_prim.answer - 1.0) < 1e-8);
    cout.precision(6);
    cout.setf(std::ios::fixed);
    cout << "kd-tree" << "," << DIM << "," << points.size() << "," << distribution << "," << result_kd.time << "\n";
    cout << "prim" << "," << DIM << "," << points.size() << "," << distribution << "," << result_prim.time << "\n";
    cout.flush();
}

template<size_t DIM>
void run_becnhmarks(std::ostream & cout) {
    run_becnhmarks<DIM>(generate_points_uniform<DIM>(10), 100, cout, "uniform");
    run_becnhmarks<DIM>(generate_points_uniform<DIM>(50), 100, cout, "uniform");
    run_becnhmarks<DIM>(generate_points_uniform<DIM>(100), 1, cout, "uniform");
    run_becnhmarks<DIM>(generate_points_uniform<DIM>(500), 1, cout, "uniform");
    run_becnhmarks<DIM>(generate_points_uniform<DIM>(1000), 1, cout, "uniform");
    run_becnhmarks<DIM>(generate_points_uniform<DIM>(5000), 1, cout, "uniform");
    run_becnhmarks<DIM>(generate_points_uniform<DIM>(10000), 1, cout, "uniform");
    run_becnhmarks<DIM>(generate_points_uniform<DIM>(50000), 1, cout, "uniform");
    /*run_becnhmarks<DIM>(generate_points_uniform<DIM>(100000), 1, cout, "uniform");
    run_becnhmarks<DIM>(generate_points_uniform<DIM>(1000000), 1, cout, "uniform");
    run_becnhmarks<DIM>(generate_points_uniform<DIM>(10000000), 1, cout, "uniform");*/
}