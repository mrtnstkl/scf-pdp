/**
 * @file geneticalgorithm.h
 *
 * Baseline Genetic Algorithm (GA) for the SCF-PDP assignment codebase.
 *
 * ---------------------------------------------------------------------------
 * What is a Genetic Algorithm?
 * ---------------------------------------------------------------------------
 * A GA is a population-based optimisation method inspired by biological
 * evolution.
 *
 * - We keep a POPULATION of candidate solutions ("individuals").
 * - Each individual has a GENOME (also called genotype / chromosome), which is
 *   just data we can recombine and mutate.
 * - A DECODER turns that genome into an actual problem solution (phenotype).
 * - We EVALUATE each solution with an objective/cost function.
 * - The best individuals (ELITES) survive, and new individuals are created by:
 *     - CROSSOVER (recombining two parents)
 *     - MUTATION (small random changes)
 * - Repeating this loop over many generations tends to improve the best cost.
 *
 * ---------------------------------------------------------------------------
 * Important note about this baseline implementation
 * ---------------------------------------------------------------------------
 * This GA is deliberately "safe and simple":
 * - It uses a Random-Key encoding (BRKGA-style): two arrays of floats in [0,1].
 * - The decoder assigns each selected request as a pair:
 *     pickup -> immediate dropoff
 *   This guarantees precedence (pickup before dropoff) and keeps capacity
 *   handling trivial.
 *
 * Because it does NOT allow interleaving pickups and dropoffs, solution quality
 * will not be great. But it's an easy-to-understand baseline that runs.
 *
 */


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

/**
 * @brief Genetic Algorithm heuristic.
 *
 * This type matches the framework expectations:
 * - Inherits from Algorithm so AlgorithmGui can store it as Algorithm*.
 * - Provides a public nested type Metrics and a public member metrics.
 * - Implements reset/step/done/draw/get_current_solution.
 */
struct GeneticAlgorithm : public Algorithm
{
	/**
	 * @brief One candidate in the population.
	 *
	 * An Individual is the GA's "genotype". We do NOT store a full route here.
	 * Instead we store *random keys* (floats in [0,1]) and later decode them
	 * into a concrete Solution.
	 */
	struct Individual
	{
		/**
		 * @brief Random keys that define the order in which we try to serve requests.
		 *
		 * Think of this as a permutation, but represented indirectly.
		 * Smaller value => considered earlier.
		 *
		 * Length = number of requests.
		 */
		std::vector<float> priority_key;

		/**
		 * @brief Random keys that decide WHICH requests we attempt to serve.
		 *
		 * For each request i:
		 *   - if select_key[i] < select_threshold => we select it
		 *   - otherwise we do not
		 *
		 * After that we REPAIR the selection to ensure we serve at least
		 * instance->min_requests.
		 */
		std::vector<float> select_key;

		/**
		 * @brief Objective value (cost) of the decoded solution.
		 *
		 * Lower is better.
		 * We initialise to +infinity so "any real solution" is an improvement.
		 */
		float objective = std::numeric_limits<float>::infinity();
	};

	/**
	 * @brief The problem instance this heuristic is solving.
	 *
	 * The Instance contains all data: depot, pickup/dropoff nodes, distances,
	 * capacities, and the minimum number of requests that must be served.
	 */
	Instance* instance = nullptr;

	/**
	 * @brief Metrics type used by the GUI/visualiser.
	 *
	 * The visualiser stores a time series of these. See util/visualization.h.
	 */
	using Metrics = SolutionMetrics;

	/**
	 * @brief Current metrics snapshot of the best-so-far solution.
	 *
	 * The GUI reads this directly.
	 */
	Metrics metrics{};

	/**
	 * @name GA parameters
	 * These control the behaviour of the evolutionary search.
	 *
	 * You can tweak these live in ImGui via draw().
	 */
	///@{
	int population_size = 120;        ///< Number of individuals per generation.
	float elite_fraction = 0.20f;     ///< Fraction of population copied unchanged.
	float mutant_fraction = 0.10f;    ///< Fraction replaced by completely random individuals.
	float elite_bias = 0.70f;         ///< In crossover: probability to inherit gene from elite parent.

	int generations_per_step = 1;     ///< How many GA generations to run per GUI "step()".
	int max_generations = 1000;       ///< Hard stop after this many generations.

	float select_threshold = 0.55f;   ///< Request i is selected if select_key[i] < threshold (before repair).
	int mutate_requests = 2;          ///< How many request-genes to perturb per child.
	float mutate_sigma = 0.25f;       ///< Stddev for gaussian noise added to keys.
	///@}

	/**
	 * @name Internal state
	 * These fields represent the current search state.
	 */
	///@{
	bool finished = false;   ///< True when the algorithm decides it is done.
	size_t generation = 0;   ///< How many GA generations have been executed.
	size_t best_step = 0;    ///< A monotonically increasing counter for progress.

	std::mt19937 rng;                 ///< Random number generator.
	std::vector<Individual> population;///< Current population.

	Individual best_individual;       ///< Genome of the best-so-far individual.
	Solution best_solution;           ///< Decoded best-so-far phenotype.
	///@}

	/**
	 * @brief Construct a GA solver for a given instance.
	 *
	 * Seeds the RNG and calls reset() to create an initial population.
	 */
	GeneticAlgorithm(Instance* instance);

	/**
	 * @brief Perform a small amount of work and return the current step counter.
	 *
	 * In this codebase, the GUI calls step() repeatedly. We interpret one step as
	 * running @ref generations_per_step GA generations.
	 */
	size_t step() override;

	/**
	 * @brief Reset internal state and start again from a fresh random population.
	 */
	void reset() override;

	/**
	 * @brief True if the algorithm has terminated.
	 */
	bool done() override;

	/**
	 * @brief Draw ImGui controls for this algorithm.
	 */
	void draw() override;

	/**
	 * @brief Return the best solution found so far.
	 *
	 * The visualiser will call this frequently.
	 */
	Solution get_current_solution() override;

private:
	/**
	 * @name GA internals
	 * Implementation helpers.
	 */
	///@{
	// GA internals
	void initialise_population();
	Individual make_random_individual();
	Individual crossover(const Individual& elite_parent, const Individual& other_parent);
	void mutate(Individual& ind);

	// decoding & eval
	/**
	 * @brief Decode an individual's genome into a concrete Solution.
	 *
	 * This is the key idea of random-key GAs:
	 * - Crossover and mutation operate on simple floats.
	 * - The decoder is responsible for producing a valid-ish Solution.
	 *
	 * Baseline decoder behaviour:
	 * - Choose a set of requests (using select_key + repair)
	 * - Order them (using priority_key)
	 * - For each request, assign it to the first vehicle that can take the pickup
	 *   without exceeding capacity.
	 * - Append pickup then immediate dropoff.
	 */
	Solution decode_to_solution(const Individual& ind) const;

	// helpers

	/**
	 * @brief Build the ordered list of request indices this individual wants to serve.
	 *
	 * Steps:
	 * 1) Select all requests with select_key[i] < select_threshold.
	 * 2) Repair: if fewer than min_requests were selected, add more requests in
	 *    increasing order of select_key until min_requests is met.
	 * 3) Sort the final selected set by priority_key (ascending).
	 */
	std::vector<int> build_selected_requests(const Individual& ind) const;
	///@}
};

