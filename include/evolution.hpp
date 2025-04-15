#pragma once

#include "simulation.hpp" // Include Simulation
#include "controller.hpp" // Include Controller
#include "config.hpp"
#include <vector>
#include <random> // For better random number generation

// Structure to hold genome and its fitness
struct Agent {
    std::vector<float> genome;
    double fitness = -1e18; // Initialize with very low fitness

    // Comparison operator for sorting/finding best
    bool operator<(const Agent& other) const {
        return fitness < other.fitness;
    }
};

class Evolution {
public:
    // Constructor takes simulation environment and controller type
    Evolution(Simulation& sim, Controller& ctrl);
    ~Evolution() = default;

    // Run the evolutionary process for NUM_GENERATIONS
    Agent run_evolution();

private:
    // --- Core EA Functions ---
    void initialize_population();
    void evaluate_population(); // Parallel evaluation
    void selection();           // Select parents for the next generation
    void crossover();           // Create offspring
    void mutation();            // Mutate offspring

    // --- Data Members ---
    Simulation& simulation;   // Reference to the simulation environment
    Controller& controller;   // Reference to the controller logic
    int genome_length;        // Number of genes (== num controls)
    std::vector<Agent> population; // Current population
    std::vector<Agent> offspring;  // Temporary storage for new generation

    // Random number generation
    std::mt19937 random_engine; // Mersenne Twister engine
    std::uniform_real_distribution<float> uniform_dist; // For general random floats [0,1]
    std::uniform_real_distribution<float> gene_dist;    // For initial gene values [-1, 1]
    std::normal_distribution<float> mutation_dist; // For mutation offset

    // Helper for tournament selection
    int tournament_selection();
};