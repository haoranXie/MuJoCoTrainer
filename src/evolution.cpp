#include "controller.cpp"

Controller::Controller(int num_joints) : joint_count(num_joints) {}

void Controller::apply(mjModel* m, mjData* d, const std::vector<float>& genome) {
    for (int i = 0; i < joint_count; ++i) {
        d->ctrl[i] = genome[i];  // assumes direct mapping
    }
}
