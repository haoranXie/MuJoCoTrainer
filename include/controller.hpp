#pragma once
#include <vector>
#include <mujoco/mujoco.h>

class Controller {
public:
    Controller(int num_joints);
    // Applies the genome directly as control signals
    void apply(mjData* d, const std::vector<float>& genome);
    // Can be extended later for more complex controllers (e.g., neural networks)
    // virtual void apply(mjModel* m, mjData* d, const std::vector<float>& genome) = 0;

private:
    int joint_count;
};