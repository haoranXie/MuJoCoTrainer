#pragma once
#include <vector>
#include <mujoco/mujoco.h>

class Controller {
public:
    Controller(int num_joints);
    void apply(mjModel* m, mjData* d, const std::vector<float>& genome);

private:
    int joint_count;
};
