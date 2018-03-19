#ifndef CIMAGEKERNELS_H
#define CIMAGEKERNELS_H
#include <array>
#include <vector>

static const std::vector<int> VsobelX { -1, 0, 1, -2, 0, 2, -1, 0, 1};
static const std::vector<int> VsobelY { 1, 2, 1, 0, 0, 0, -1, -2, -1};

static const std::vector<int> VprewittX{ -1, 0, 1, -1, 0, 1, -1, 0, 1 };
static const std::vector<int> VprewittY { -1, -1, -1, 0, 0, 0, 1, 1, 1 };

static const std::vector<int> VrobertX { 1, 0,0, -1 };
static const std::vector<int> VrobertY { 0, 1, -1, 0 };

static const std::vector<int> VscharrX { 3, 10, 3, 0, 0, 0, -3, -10, -3 };
static const std::vector<int> VscharrY { 3, 0, -3, 10, 0, -10 , 3, 0, -3 };

#endif // CIMAGEKERNELS_H
