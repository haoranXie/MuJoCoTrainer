#include "controller.hpp"
#include <stdexcept>
#include <vector>

Controller::Controller(int num_joints) : joint_count(num_joints) {
    if (num_joints <= 0) {
        throw std::invalid_argument("Number of joints must be positive.");
    }
}

void Controller::apply(mjData* d, const std::vector<float>& genome) {
    if (genome.size() != joint_count) {
        throw std::runtime_error("Genome size does not match controller joint count.");
    }
    if (!d || !d->ctrl) {
        throw std::runtime_error("MuJoCo data or control array is null in Controller::apply.");
    }
    // Directly map genome values to control signals
    // Ensure genome values are within MuJoCo's control range if necessary (often [-1, 1])
    for (int i = 0; i < joint_count; ++i) {
        // Clamp controls if model has specific ranges (optional, depends on model)
        // float ctrl_range = m->actuator_ctrlrange[i * 2 + 1]; // Example if range is symmetric
        // d->ctrl[i] = std::max(-ctrl_range, std::min(ctrl_range, genome[i]));
        d->ctrl[i] = genome[i]; // Assuming genome is already in appropriate range [-1, 1]
    }
}