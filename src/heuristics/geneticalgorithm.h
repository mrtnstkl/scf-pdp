#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include "core/instance.h"
#include "core/solution.h"
#include "core/algorithm.h"
#include "core/intermediary.h"

struct GeneticAlgorithm : public Algorithm
{
	struct Individual
	{
		// Random keys (BRKGA-style):
		// - priority_key: order of requests
		// - select_key: which requests we try to serve (repaired to >= min_requests)
		std::vector<float> priority_key;
		std::vector<float> select_key;
		float objective = std::numeric_limits<float>::infinity();
	};

	Instance* instance = nullptr;

	using Metrics = SolutionMetrics;
	Metrics metrics{};

	// parameters (tweakable in draw())
	int population_size = 120;
	float elite_fraction = 0.20f;
	float mutant_fraction = 0.10f;
	float elite_bias = 0.70f;

	int generations_per_step = 1;
	int max_generations = 1000;

	float select_threshold = 0.55f;
	int mutate_requests = 2;
	float mutate_sigma = 0.25f;

	// state
	bool finished = false;
	size_t generation = 0;
	size_t best_step = 0;

	std::mt19937 rng;
	std::vector<Individual> population;

	Individual best_individual;
	Solution best_solution; // no default ctor -> init in ctor init-list

	GeneticAlgorithm(Instance* instance);

	size_t step() override;
	void reset() override;
	bool done() override;

	void draw() override;

	Solution get_current_solution() override;

private:
	// GA internals
	void initialise_population();
	Individual make_random_individual();
	Individual crossover(const Individual& elite_parent, const Individual& other_parent);
	void mutate(Individual& ind);

	// decoding & eval
	Solution decode_to_solution(const Individual& ind) const;

	// helpers
	size_t request_count() const;
	std::vector<int> build_selected_requests(const Individual& ind) const;
};
