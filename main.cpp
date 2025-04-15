#include <mujoco/mujoco.h>
#include <iostream>
#include <fstream>
#include <vector>

int main() {
    const char* modelPath = "mujoco_models/humanoid.xml";
    mjModel* m = mj_loadXML(modelPath, nullptr, nullptr, 0);
    if (!m) {
        std::cerr << "Could not load model!" << std::endl;
        return 1;
    }

    mjData* d = mj_makeData(m);
    for (int i = 0; i < 1000; ++i) {
        mj_step(m, d);
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
