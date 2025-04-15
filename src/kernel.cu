#include "kernel.cuh"

__global__
void evaluate_genomes_kernel(float* genomes, float* fitnesses, int genome_length, int pop_size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= pop_size) return;

    float fitness = 0.0f;
    for (int i = 0; i < genome_length; ++i) {
        float gene = genomes[idx * genome_length + i];
        fitness += gene * gene;
    }

    fitnesses[idx] = -fitness;  // Higher fitness is better
}

void launch_evaluation_kernel(float* genomes, float* fitnesses, int genome_length, int pop_size) {
    int threads = 256;
    int blocks = (pop_size + threads - 1) / threads;
    evaluate_genomes_kernel <<<blocks, threads >>> (genomes, fitnesses, genome_length, pop_size);
    cudaDeviceSynchronize();
}
