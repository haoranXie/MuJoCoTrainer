#include "evolution.hpp"
#include "kernel.cuh"
#include <cuda_runtime.h>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <ctime>

Evolution::Evolution(int population_size, int genome_length)
    : pop_size(population_size), gene_len(genome_length), d_genomes(nullptr), d_fitnesses(nullptr) {
    cudaMalloc(&d_genomes, pop_size * gene_len * sizeof(float));    
    cudaMalloc(&d_fitnesses, pop_size * sizeof(float));
    std::srand(static_cast<unsigned>(std::time(nullptr)));
}

Evolution::~Evolution() {
    cudaFree(d_genomes);
    cudaFree(d_fitnesses);
}

std::vector<float> Evolution::optimize() {
    std::vector<float> genomes(pop_size * gene_len);
    for (auto& gene : genomes) {
        gene = static_cast<float>(std::rand()) / RAND_MAX * 2.0f - 1.0f;  // range [-1, 1]
    }

    cudaMemcpy(d_genomes, genomes.data(), pop_size * gene_len * sizeof(float), cudaMemcpyHostToDevice);

    int threads = 256;
    int blocks = (pop_size + threads - 1) / threads;
    launch_evaluation_kernel(d_genomes, d_fitnesses, gene_len, pop_size);


    std::vector<float> fitnesses(pop_size);
    cudaMemcpy(fitnesses.data(), d_fitnesses, pop_size * sizeof(float), cudaMemcpyDeviceToHost);

    int best_idx = 0;
    for (int i = 1; i < pop_size; ++i) {
        if (fitnesses[i] > fitnesses[best_idx]) {
            best_idx = i;
        }
    }

    return std::vector<float>(
        genomes.begin() + best_idx * gene_len,
        genomes.begin() + (best_idx + 1) * gene_len
    );
}
