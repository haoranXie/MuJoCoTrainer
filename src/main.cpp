#include "controller.hpp"
#include "evolution.hpp"
#include <mujoco/mujoco.h>
#include <iostream>
#include <vector>
#include <string>

int main() {

    char error[1000] = "";
    mjModel* m = mj_loadXML("mujoco_models/humanoid.xml", nullptr, error, 1000);
    if (!m) {
        std::cerr << "Could not load MuJoCo model: " << error << std::endl;
        return -1;
    }

    mjData* d = mj_makeData(m);

    int num_joints = m->nu;  // nu = number of control inputs
    Controller controller(num_joints);
    Evolution evolution(100, num_joints);

    std::vector<float> best_genome = evolution.optimize();
    controller.apply(m, d, best_genome);

    while (d->time < 2.0) {
        mj_step(m, d);
        std::cout << "Time: " << d->time << " Control: ";
        for (int i = 0; i < num_joints; ++i) {
            std::cout << d->ctrl[i] << " ";
        }
        std::cout << std::endl;
    }

    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}
