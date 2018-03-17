#ifndef CIMAGEKERNELS_H
#define CIMAGEKERNELS_H
#include <array>
#include <vector>

//template<class T, size_t Rows, size_t Cols> using matrix = std::array<std::array<T, Cols>, Rows>;

//const matrix<int, 3, 3> sobelx {{ {{-1, 0, 1}}, {{-2, 0, 2}}, {{-1, 0, 1}} }};
//const matrix<int, 3, 3> sobely {{ {{1, 2, 1}}, {{0, 0, 0}}, {{-1, -2, -1}} }};
//const matrix<int, 3, 3> prewittx {{ {{-1, 0, 1}}, {{-1, 0, 1}}, {{-1, 0, 1}} }};
//const matrix<int, 3, 3> prewitty {{ {{-1, -1, -1}}, {{0, 0, 0}}, {{1, 1, 1}} }};
//const matrix<int, 2, 2> robertsx {{ {{1, 0}}, {{0, -1}} }};
//const matrix<int, 2, 2> robertsy {{ {{0, 1}}, {{-1, 0}} }};
//const matrix<int, 3, 3> scharrx {{ {{3, 10, 3}}, {{0, 0, 0}}, {{-3, -10, -3}} }};
//const matrix<int, 3, 3> scharry {{ {{3, 0, -3}}, {{10, 0, -10}}, {{3, 0, -3}} }};

static const std::vector<int> VsobelX { -1, 0, 1, -2, 0, 2, -1, 0, 1};
static const std::vector<int> VsobelY { 1, 2, 1, 0, 0, 0, -1, -2, -1};

#endif // CIMAGEKERNELS_H
