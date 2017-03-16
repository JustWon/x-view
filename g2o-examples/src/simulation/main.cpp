#include "Simulator.hpp"

#include <Eigen/Core>

int main() {

    Simulator simulator;

    int numPoses = 100;
    int steps = 4;

    const Eigen::Vector2d transNoise(0.02, 0.04);
    const double rotNoise(DEG2RAD(3));
    const Eigen::Vector2d bound(20, 20);

    simulator.simulate(numPoses, steps, transNoise, rotNoise, bound);
    return 0;
}
