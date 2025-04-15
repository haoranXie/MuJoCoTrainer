#pragma once
#include <vector>

struct Genome {
    std::vector<float> genes;
    float fitness;
};

class Evolution {
public:
    Evolution(int population_size, int genome_length);
    void evaluate_all(); // calls CUDA
    void evolve();
    const std::vector<Genome>& get_population() const;

private:
    int pop_size;
    int gene_len;
    std::vector<Genome> population;
};
