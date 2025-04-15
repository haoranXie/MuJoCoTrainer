#pragma once

#include "controller.hpp"
#include "config.hpp"
#include <mujoco/mujoco.h>
#include <vector>
#include <string>
#include <memory> // For unique_ptr

// Holds MuJoCo model and provides simulation functionality
class Simulation {
public:
    // Loads the model (throws runtime_error on failure)
    Simulation(const std::string& model_path);
    ~Simulation();

    // Non-copyable
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    // Runs one simulation instance with the given genome and returns fitness
    // Takes a controller reference to apply the genome
    double evaluate_fitness(const std::vector<float>& genome, Controller& controller);

    int get_num_controls() const;
    const mjModel* get_model_ptr() const; // Allow access for specific needs

private:
    mjModel* m = nullptr; // The master model, loaded once

    // Helper to get torso Z position (example fitness component)
    double get_torso_z_position(const mjData* d) const;
    // Helper to get center of mass X velocity (example fitness component)
    double get_com_x_velocity(const mjData* d) const;
};