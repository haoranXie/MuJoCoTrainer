#pragma once

// Evolution Parameters
constexpr int POPULATION_SIZE = 100; // Number of individuals in the population
constexpr int NUM_GENERATIONS = 50; // Number of generations to run
constexpr double MUTATION_RATE = 0.1;  // Probability of mutating a single gene
constexpr double MUTATION_STRENGTH = 0.2; // Magnitude of mutation
constexpr double CROSSOVER_RATE = 0.7; // Probability of performing crossover
constexpr int TOURNAMENT_SIZE = 5;   // Size for tournament selection
constexpr bool USE_ELITISM = true;  // Keep the best individual from the previous generation

// Simulation Parameters
constexpr double SIMULATION_TIME = 5.0; // Duration of each fitness evaluation simulation (seconds)
constexpr double TIMESTEP = 0.002;       // MuJoCo simulation timestep (if different from model default)
constexpr const char* MODEL_PATH = "mujoco_models/humanoid.xml"; // Path to MuJoCo model

// Fitness Function Parameters (Example: Maximize forward movement)
constexpr double FORWARD_VELOCITY_WEIGHT = 1.0;
constexpr double UPRIGHT_BONUS_WEIGHT = 0.5;
constexpr double CONTROL_COST_WEIGHT = 0.01;
constexpr double FALL_PENALTY = -100.0; // Penalty if torso height drops too low
constexpr double MIN_HEIGHT_THRESHOLD = 0.8; // Minimum acceptable torso height (example for humanoid)