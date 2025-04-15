#include "simulation.hpp"
#include "config.hpp"
#include <mujoco/mujoco.h>
#include <iostream>
#include <stdexcept>
#include <cmath> // For std::sqrt, std::abs
#include <numeric> // For std::accumulate

Simulation::Simulation(const std::string& model_path) {
    char error[1000] = "Could not load model";
    m = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
    if (!m) {
        throw std::runtime_error("MuJoCo load error: " + std::string(error));
    }
    // Optional: Override timestep if needed
    // m->opt.timestep = TIMESTEP;
}

Simulation::~Simulation() {
    if (m) {
        mj_deleteModel(m);
        m = nullptr;
    }
}

int Simulation::get_num_controls() const {
    if (!m) return 0;
    return m->nu; // Number of actuators/controls
}

const mjModel* Simulation::get_model_ptr() const {
    return m;
}

// --- Fitness Component Helpers (Example for Humanoid) ---

double Simulation::get_torso_z_position(const mjData* d) const {
    // Find the torso body ID (assuming it's named "torso")
    int torso_id = mj_name2id(m, mjOBJ_BODY, "torso");
    if (torso_id < 0 || !d) {
        // std::cerr << "Warning: Could not find 'torso' body for height calculation." << std::endl;
        return 0.0; // Or handle error appropriately
    }
    return d->xpos[torso_id * 3 + 2]; // Z position of the torso
}

double Simulation::get_com_x_velocity(const mjData* d) const {
    if (!d) return 0.0;
    // Use subtree center of mass velocity (more stable than body velocity)
    // Assumes the root body (world) is ID 0, calculate for subtree starting at root child (e.g., pelvis)
    // If your model's base link is different, adjust accordingly.
    if (m->nbody > 1) {
        // Use xvel (6D: 3 linear, 3 angular). Index 0 is linear X velocity.
        // Often need COM velocity, which mj_comPos/Vel computes, but requires mj_kinematics first
        // A simpler approximation: use velocity of a major body like the torso
        int torso_id = mj_name2id(m, mjOBJ_BODY, "torso");
        if (torso_id >= 0) {
            return d->cvel[torso_id * 6 + 0]; // Linear velocity X of torso frame
        }
    }
    // Fallback or alternative: Compute overall COM velocity manually if needed
   // This is a placeholder; precise COM velocity might need mj_subtreeVel
    return 0.0;
}


// --- Main Evaluation Function ---

double Simulation::evaluate_fitness(const std::vector<float>& genome, Controller& controller) {
    if (!m) {
        throw std::runtime_error("Model not loaded in Simulation::evaluate_fitness");
    }

    // Create thread-local data instance for this evaluation
    // Using unique_ptr for automatic cleanup
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m), mj_deleteData);
    if (!d) {
        throw std::runtime_error("Could not create mjData in Simulation::evaluate_fitness");
    }

    // Reset simulation state to initial conditions
    mj_resetData(m, d.get());

    double total_forward_velocity = 0.0;
    double total_control_cost = 0.0;
    double min_torso_height = 1000.0; // Start high
    int step_count = 0;
    bool fallen = false;

    // Simulation loop for fitness evaluation
    while (d->time < SIMULATION_TIME) {
        // Apply controls based on the genome
        try {
            controller.apply(d.get(), genome);
        }
        catch (const std::exception& e) {
            std::cerr << "Error applying controls: " << e.what() << std::endl;
            return -1e9; // Penalize heavily for errors during control
        }

        // Step the simulation
        mj_step(m, d.get());

        // --- Calculate Fitness Components ---
        double current_x_vel = get_com_x_velocity(d.get());
        total_forward_velocity += current_x_vel * m->opt.timestep; // Integrate velocity

        double current_z_pos = get_torso_z_position(d.get());
        min_torso_height = std::min(min_torso_height, current_z_pos);

        // Control cost (penalize large control signals)
        double sq_ctrl_sum = 0.0;
        for (int i = 0; i < m->nu; ++i) {
            sq_ctrl_sum += d->ctrl[i] * d->ctrl[i];
        }
        total_control_cost += std::sqrt(sq_ctrl_sum) * m->opt.timestep;

        // Check for fall condition
        if (current_z_pos < MIN_HEIGHT_THRESHOLD) {
            fallen = true;
            break; // End simulation early if fallen
        }

        step_count++;
    }

    // --- Combine Fitness Components ---
    double fitness = 0.0;

    fitness += FORWARD_VELOCITY_WEIGHT * (total_forward_velocity / SIMULATION_TIME); // Average forward velocity
    fitness -= CONTROL_COST_WEIGHT * (total_control_cost / SIMULATION_TIME); // Average control cost

    // Bonus for staying upright (using minimum height achieved)
    if (min_torso_height > MIN_HEIGHT_THRESHOLD) {
        fitness += UPRIGHT_BONUS_WEIGHT * min_torso_height;
    }

    // Heavy penalty for falling
    if (fallen) {
        fitness += FALL_PENALTY;
    }

    // Add more components as needed (e.g., energy efficiency, stability, target reaching)

    // Handle NaN or infinite fitness values
    if (!std::isfinite(fitness)) {
        std::cerr << "Warning: Non-finite fitness (" << fitness << ") detected for a genome. Penalizing." << std::endl;
        fitness = -1e9; // Assign a very poor fitness score
    }


    return fitness;
}