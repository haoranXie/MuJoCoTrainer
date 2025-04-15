#pragma once
#include <vector>

struct Genome {
    std::vector<float> genes;
    float fitness;
};

class Evolution {
public:
    Evolution(int population_size, int genome_length);
    ~Evolution();
    std::vector<float> optimize();

private:
    int pop_size;
    int gene_len;
    float* d_genomes;
    float* d_fitnesses;
};
