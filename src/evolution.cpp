#include "evolution.hpp"
#include "config.hpp"
#include <vector>
#include <iostream>
#include <algorithm> // For std::sort, std::max_element, std::generate
#include <random>
#include <stdexcept>
#include <chrono> // For seeding random engine
#include <omp.h>  // OpenMP for parallelism

Evolution::Evolution(Simulation& sim, Controller& ctrl)
    : simulation(sim),
    controller(ctrl),
    genome_length(sim.get_num_controls()),
    population(POPULATION_SIZE),
    offspring(POPULATION_SIZE),
    // Seed with time for better randomness
    random_engine(std::chrono::high_resolution_clock::now().time_since_epoch().count()),
    uniform_dist(0.0f, 1.0f),
    gene_dist(-1.0f, 1.0f), // Initial genes in [-1, 1] range
    mutation_dist(0.0f, MUTATION_STRENGTH) // Normal distribution for mutation
{
    if (genome_length <= 0) {
        throw std::runtime_error("Genome length must be positive (check MuJoCo model controls).");
    }
    if (POPULATION_SIZE <= 0) {
        throw std::runtime_error("Population size must be positive.");
    }
    initialize_population();
}

void Evolution::initialize_population() {
    std::cout << "Initializing population (" << POPULATION_SIZE << " agents, "
        << genome_length << " genes)..." << std::endl;
    for (Agent& agent : population) {
        agent.genome.resize(genome_length);
        // Generate random genes within the typical MuJoCo control range [-1, 1]
        std::generate(agent.genome.begin(), agent.genome.end(),
            [&]() { return gene_dist(random_engine); });
        agent.fitness = -1e18; // Mark as unevaluated
    }
    std::cout << "Initialization complete." << std::endl;
}

void Evolution::evaluate_population() {
    // Use OpenMP to parallelize the fitness evaluation loop
    // Each thread will call simulation.evaluate_fitness
    // Ensure Simulation::evaluate_fitness is thread-safe (it should be if
    // mjData is created locally within the function as done in the refactored version).
#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < POPULATION_SIZE; ++i) {
        // Each thread works on a different agent 'i'
        population[i].fitness = simulation.evaluate_fitness(population[i].genome, controller);

        // Optional: Print progress from one thread to avoid spam
        // #pragma omp critical
        // {
        //     if (i % (POPULATION_SIZE / 10) == 0 && i > 0) { // Print roughly 10 updates
        //         std::cout << "  Evaluated " << i << "/" << POPULATION_SIZE << " agents..." << std::endl;
        //     }
        // }
    }
    // Ensure all fitnesses are calculated before proceeding
}

// Tournament selection implementation
int Evolution::tournament_selection() {
    int best_index = -1;
    double best_fitness = -1e18; // Start with very low fitness

    for (int i = 0; i < TOURNAMENT_SIZE; ++i) {
        // Select a random individual from the population
        std::uniform_int_distribution<int> index_dist(0, POPULATION_SIZE - 1);
        int random_index = index_dist(random_engine);

        if (best_index == -1 || population[random_index].fitness > best_fitness) {
            best_fitness = population[random_index].fitness;
            best_index = random_index;
        }
    }
    return best_index;
}


void Evolution::selection() {
    // Prepare the offspring vector for the new generation
    offspring.clear();
    offspring.reserve(POPULATION_SIZE);

    // Elitism: Keep the best individual(s) directly if enabled
    int elite_count = 0;
    if (USE_ELITISM) {
        // Find the best agent in the current population
        auto best_it = std::max_element(population.begin(), population.end());
        if (best_it != population.end()) {
            offspring.push_back(*best_it); // Add the best agent to the offspring
            elite_count = 1;
            // std::cout << "  Elite Fitness: " << best_it->fitness << std::endl; // Debug
        }
    }

    // Fill the rest of the offspring population using tournament selection
    while (offspring.size() < POPULATION_SIZE) {
        int parent1_idx = tournament_selection();
        int parent2_idx = tournament_selection();

        // Ensure parents are different if possible, though not strictly required
        // while (parent2_idx == parent1_idx && POPULATION_SIZE > 1) {
        //     parent2_idx = tournament_selection();
        // }

        // Add selected parents (or placeholders for their future children)
        // We will perform crossover/mutation on this 'offspring' vector next
        offspring.push_back(population[parent1_idx]); // Placeholder child 1 (will be modified)
        if (offspring.size() < POPULATION_SIZE) {
            offspring.push_back(population[parent2_idx]); // Placeholder child 2
        }
    }

    // Ensure offspring vector has the correct size (handling edge case of odd population size with elitism)
    offspring.resize(POPULATION_SIZE);
}


void Evolution::crossover() {
    // Start crossover from index 'elite_count' if elitism is used
    int start_index = USE_ELITISM ? 1 : 0;

    // Process pairs of parents (now stored sequentially in 'offspring' after selection)
    for (int i = start_index; i < POPULATION_SIZE - 1; i += 2) { // Step by 2
        if (uniform_dist(random_engine) < CROSSOVER_RATE) {
            // Perform single-point crossover (example)
            std::uniform_int_distribution<int> crossover_point_dist(1, genome_length - 2); // Avoid edges
            int crossover_point = crossover_point_dist(random_engine);

            Agent& child1 = offspring[i];
            Agent& child2 = offspring[i + 1];

            // Swap genes after the crossover point
            for (int j = crossover_point; j < genome_length; ++j) {
                std::swap(child1.genome[j], child2.genome[j]);
            }
            // Invalidate fitness as genomes have changed
            child1.fitness = -1e18;
            child2.fitness = -1e18;
        }
        // If no crossover, the offspring keep the genomes selected in the previous step
        // Their fitness also needs invalidation if they weren't the elite one.
        else {
            if (i >= start_index) offspring[i].fitness = -1e18;
            if (i + 1 >= start_index) offspring[i + 1].fitness = -1e18;
        }
    }
    // Handle the last individual if population size is odd and elitism is off
    if (start_index == 0 && POPULATION_SIZE % 2 != 0) {
        offspring[POPULATION_SIZE - 1].fitness = -1e18;
    }
}

void Evolution::mutation() {
    int start_index = USE_ELITISM ? 1 : 0; // Don't mutate the elite individual

    for (int i = start_index; i < POPULATION_SIZE; ++i) {
        Agent& agent = offspring[i];
        for (int j = 0; j < genome_length; ++j) {
            if (uniform_dist(random_engine) < MUTATION_RATE) {
                // Add a small random value (Gaussian mutation)
                float mutation_value = mutation_dist(random_engine);
                agent.genome[j] += mutation_value;

                // Optional: Clamp the gene value to a valid range (e.g., [-1, 1])
                agent.genome[j] = std::max(-1.0f, std::min(1.0f, agent.genome[j]));

                agent.fitness = -1e18; // Invalidate fitness after mutation
            }
        }
    }
}

Agent Evolution::run_evolution() {
    std::cout << "\n--- Starting Evolution ---" << std::endl;
    Agent best_overall_agent;
    best_overall_agent.fitness = -1e18;

    for (int gen = 0; gen < NUM_GENERATIONS; ++gen) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // 1. Evaluate Fitness
        std::cout << "\nGeneration " << gen + 1 << "/" << NUM_GENERATIONS << std::endl;
        std::cout << " Evaluating population..." << std::endl;
        evaluate_population();

        // Find and report best fitness in current generation
        auto current_best_it = std::max_element(population.begin(), population.end());
        double current_best_fitness = -1e18;
        if (current_best_it != population.end()) {
            current_best_fitness = current_best_it->fitness;
            if (current_best_fitness > best_overall_agent.fitness) {
                best_overall_agent = *current_best_it; // Update overall best
            }
        }


        // 2. Selection
        // std::cout << " Performing selection..." << std::endl;
        selection(); // Selects parents and places them (or elite) in offspring vector

        // 3. Crossover
        // std::cout << " Performing crossover..." << std::endl;
        crossover(); // Modifies genomes in offspring vector

        // 4. Mutation
        // std::cout << " Performing mutation..." << std::endl;
        mutation(); // Modifies genomes in offspring vector

        // 5. Replace Population
        population = offspring; // The modified offspring become the new population

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;

        std::cout << " Generation " << gen + 1 << " finished. Best Fitness: " << current_best_fitness
            << " (Overall Best: " << best_overall_agent.fitness << ")"
            << " Time: " << elapsed.count() << "s" << std::endl;

        // Optional: Add termination condition based on fitness stagnation or target
        // if (best_overall_agent.fitness > SOME_TARGET_FITNESS) {
        //      std::cout << "Target fitness reached. Stopping evolution." << std::endl;
        //      break;
        // }

    }

    std::cout << "\n--- Evolution Finished ---" << std::endl;
    std::cout << "Best overall fitness achieved: " << best_overall_agent.fitness << std::endl;
    // You might want to save the best genome here:
    // std::cout << "Best genome: ";
    // for(float gene : best_overall_agent.genome) { std::cout << gene << " "; }
    // std::cout << std::endl;

    return best_overall_agent;
}