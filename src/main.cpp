#include "config.hpp"
#include "controller.hpp"
#include "evolution.hpp"
#include "simulation.hpp" // Include Simulation
#include <mujoco/mujoco.h>
#include <iostream>
#include <vector>
#include <string>
#include <stdexcept> // For exception handling
#include <memory> // For unique_ptr

// Basic GLFW example includes - for optional visualization later
// #include <GLFW/glfw3.h>
// #include "uitools.h" // If using MuJoCo's sample UI tools

// --- Simulation Visualization Function (Optional) ---
/*
void visualize_simulation(mjModel* m, const std::vector<float>& best_genome) {
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
        return;
    }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
    if (!window) {
        glfwTerminate();
        mju_error("Could not create GLFW window");
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);

    mjr_makeContext(m, &con, mjFONTSCALE_150);
    mjv_makeScene(m, &scn, 2000);

    mjData* d = mj_makeData(m);
    Controller controller(m->nu);

    while (!glfwWindowShouldClose(window) && d->time < 15.0) { // Run for 15 seconds
         mjtime simstart = d->time;
         while (d->time - simstart < 1.0/60.0) { // Aim for 60 Hz simulation rate
            controller.apply(d, best_genome);
            mj_step(m, d);
         }

        // Get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // Update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // Swap OpenGL buffers
        glfwSwapBuffers(window);
        glfwPollEvents(); // Process window events
    }

    mj_deleteData(d);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwTerminate();
}
*/

int main() {
    try {
        std::cout << "Loading MuJoCo model from: " << MODEL_PATH << std::endl;
        Simulation simulation(MODEL_PATH); // Handles model loading/cleanup

        int num_joints = simulation.get_num_controls();
        if (num_joints <= 0) {
            std::cerr << "Error: Model has no controllable joints (nu=0)." << std::endl;
            return -1;
        }
        std::cout << "Model loaded successfully. Number of controllable joints: " << num_joints << std::endl;

        // Create the controller instance
        Controller controller(num_joints);

        // Create the evolution instance
        Evolution evolution(simulation, controller);

        // Run the evolutionary process
        Agent best_agent = evolution.run_evolution();

        std::cout << "\n--- Final Result ---" << std::endl;
        std::cout << "Best fitness found: " << best_agent.fitness << std::endl;
        std::cout << "Best genome control values:" << std::endl;
        for (size_t i = 0; i < best_agent.genome.size(); ++i) {
            std::cout << "  Joint " << i << ": " << best_agent.genome[i] << std::endl;
        }

        // --- Optional: Visualize the best controller ---
        /*
        std::cout << "\nRunning visualization with the best genome..." << std::endl;
        // Need the model pointer from the simulation object for visualization
        const mjModel* m_ptr = simulation.get_model_ptr();
        if (m_ptr) {
             visualize_simulation(const_cast<mjModel*>(m_ptr), best_agent.genome);
        } else {
            std::cerr << "Could not get model pointer for visualization." << std::endl;
        }
        */
        // --- End Optional Visualization ---


    }
    catch (const std::runtime_error& e) {
        std::cerr << "Runtime Error: " << e.what() << std::endl;
        return -1;
    }
    catch (const std::exception& e) {
        std::cerr << "Standard Exception: " << e.what() << std::endl;
        return -1;
    }
    catch (...) {
        std::cerr << "An unknown error occurred." << std::endl;
        return -1;
    }

    std::cout << "\nProgram finished successfully." << std::endl;
    return 0;
}