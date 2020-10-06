#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <string>
#include "emst/emst.hpp"
using namespace std;

template<size_t DIM>
void example() {

    fstream fin("../test_data/dim" + to_string(DIM) + ".txt");
    size_t n; fin >> n;
    vector<Point<DIM>> points(n);
    for (size_t i = 0; i < n; i++) {
        for (size_t k = 0; k < DIM; k++) {
            fin >> points[i][k];
        }
    }
    string s; fin >> s;
    double answer; fin >> answer;

    KdTreeSolver<DIM> solver_fast(points);
    PrimSolver<DIM> solver_slow(points);

    cout << DIM << "-dimensional space:" << endl;
    cout << "Answer: " << answer << endl;
    cout << "Fast solver answer: " << solver_fast.get_total_length() << endl;
    cout << "Slow solver answer: " << solver_slow.get_total_length() << endl;
    cout << endl;
}

int main() {
    cout.setf(ios::fixed);
    cout.precision(6);

    example<2>();
    example<10>();

    return 0;
}