extern "C" __global__
void evaluate_genomes(float* genomes, float* fitnesses, int genome_length, int pop_size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= pop_size) return;

    float fitness = 0.0f;
    for (int i = 0; i < genome_length; ++i) {
        float gene = genomes[idx * genome_length + i];
        fitness += gene * gene;  // replace with real sim
    }

    fitnesses[idx] = -fitness;  // dummy: minimize square sum
}
