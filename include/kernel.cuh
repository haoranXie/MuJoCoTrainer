#pragma once
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

extern "C" __global__
void launch_evaluation_kernel(float* genomes, float* fitnesses, int genome_length, int pop_size);
