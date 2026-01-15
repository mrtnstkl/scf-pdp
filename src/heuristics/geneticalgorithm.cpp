#include "heuristics/geneticalgorithm.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#ifndef SCFPDP_HEADLESS
#include "imgui.h"
#endif // SCFPDP_HEADLESS

/**
 * @brief Create a GA solver.
 */
GeneticAlgorithm::GeneticAlgorithm(Instance* instance)
	: instance(instance)
	, best_solution(instance) // Solution has no default ctor
{
	//std::seed_seq seq{
	//	(uint32_t)instance->vehicle_count,
	//	(uint32_t)instance->min_requests,
	//	(uint32_t)instance->vehicle_capacity,
	//	(uint32_t)instance->request_count
	//};
	//rng.seed(seq);
	rng.seed(std::random_device{}());
	reset();
}

/**
 * @brief Reset the search.
 *
 * This discards any previously found solutions and starts again with a fresh,
 * randomly initialised population.
 */
void GeneticAlgorithm::reset()
{
	finished = false;
	generation = 0;
	best_step = 0;

	population.clear();
	best_individual = Individual{};
	best_solution = Solution(instance);

	initialise_population();

	// Evaluate each individual and remember the best one.
	for (auto& ind : population)
	{
		Solution sol = decode_to_solution(ind);
		ind.objective = sol.calculate_cost();

		if (ind.objective < best_individual.objective)
		{
			best_individual = ind;
			best_solution = std::move(sol);
		}
	}

	best_solution.calculate_metrics(metrics);
	metrics.step = 0;
}

/**
 * @brief Advance the GA by a small amount of work.
 *
 * In each call we run @ref generations_per_step GA generations.
 *
 * One GA generation consists of:
 * 1) Sort population by objective (best first).
 * 2) Copy the best @ref elite_fraction unchanged into the next generation.
 * 3) Create new offspring by crossover (elite x non-elite) + mutation.
 * 4) Add a small number of brand new random individuals ("mutants") to keep
 *    diversity and reduce premature convergence.
 */
size_t GeneticAlgorithm::step()
{
	if (finished)
		return metrics.step;

	// The framework's worker can request a stop (e.g., UI button).
	if (worker.stop_requested)
	{
		finished = true;
		return metrics.step;
	}

	// Keep parameters in a sane range, even if the UI sets weird values.
	population_size = std::max(10, population_size);
	elite_fraction = std::clamp(elite_fraction, 0.01f, 0.90f);
	mutant_fraction = std::clamp(mutant_fraction, 0.00f, 0.90f);
	elite_bias = std::clamp(elite_bias, 0.50f, 0.99f);

	generations_per_step = std::max(1, generations_per_step);
	max_generations = std::max(1, max_generations);

	// If the population size changed live, we rebuild.
	if ((int)population.size() != population_size)
	{
		reset();
		return metrics.step;
	}

	const int elite_count = std::max(1, (int)std::floor(population_size * elite_fraction));
	const int mutant_count = std::max(0, (int)std::floor(population_size * mutant_fraction));
	const int offspring_count = population_size - elite_count - mutant_count;

	std::uniform_int_distribution<int> elite_pick(0, elite_count - 1);
	std::uniform_int_distribution<int> nonelite_pick(elite_count, population_size - 1);

	for (int it = 0; it < generations_per_step && !finished; ++it)
	{
		if (worker.stop_requested)
		{
			finished = true;
			break;
		}

		++generation;
		++best_step;

		// Sort: best objective first.
		std::sort(population.begin(), population.end(),
			[](const Individual& a, const Individual& b) { return a.objective < b.objective; });

		// Update global best (best-so-far) from the current generation leader.
		if (population.front().objective < best_individual.objective)
		{
			best_individual = population.front();
			best_solution = decode_to_solution(best_individual);
			best_solution.calculate_metrics(metrics);
		}

		// Stop criterion.
		if ((int)generation >= max_generations)
		{
			finished = true;
			break;
		}

		std::vector<Individual> next;
		next.reserve(population_size);

		// 1) Elites survive unchanged.
		for (int i = 0; i < elite_count; ++i)
			next.push_back(population[i]);

		// 2) Offspring generation via crossover + mutation.
		for (int i = 0; i < offspring_count; ++i)
		{
			const Individual& elite = population[elite_pick(rng)];
			const Individual& other = population[nonelite_pick(rng)];

			Individual child = crossover(elite, other);
			mutate(child);

			Solution sol = decode_to_solution(child);
			child.objective = sol.calculate_cost();

			if (child.objective < best_individual.objective)
			{
				best_individual = child;
				best_solution = sol;
				best_solution.calculate_metrics(metrics);
			}

			next.push_back(std::move(child));
		}

		// 3) Mutants: completely random individuals for diversity.
		for (int i = 0; i < mutant_count; ++i)
		{
			Individual m = make_random_individual();
			Solution sol = decode_to_solution(m);
			m.objective = sol.calculate_cost();

			if (m.objective < best_individual.objective)
			{
				best_individual = m;
				best_solution = sol;
				best_solution.calculate_metrics(metrics);
			}

			next.push_back(std::move(m));
		}

		population = std::move(next);
	}

	// "metrics.step" is the x-axis for the visualiser.
	return ++metrics.step;
}

/**
 * @brief Whether the algorithm is finished.
 */
bool GeneticAlgorithm::done()
{
	return finished;
}

/**
 * @brief Return the best solution found so far.
 *
 * Note: this returns by value (copy). That's simple and matches the existing
 * BeamSearch code pattern.
 */
Solution GeneticAlgorithm::get_current_solution()
{
	return best_solution;
}

// -------- internals --------

/**
 * @brief Create the initial random population.
 */
void GeneticAlgorithm::initialise_population()
{
	population.reserve(population_size);
	for (int i = 0; i < population_size; ++i)
		population.push_back(make_random_individual());
}

/**
 * @brief Create a brand new random individual.
 *
 * Both key arrays are filled with independent random floats in [0,1].
 */
GeneticAlgorithm::Individual GeneticAlgorithm::make_random_individual()
{
	const size_t n = (size_t)instance->request_count;
	std::uniform_real_distribution<float> U(0.0f, 1.0f);

	Individual ind;
	ind.priority_key.resize(n);
	ind.select_key.resize(n);

	for (size_t i = 0; i < n; ++i)
	{
		ind.priority_key[i] = U(rng);
		ind.select_key[i] = U(rng);
	}
	return ind;
}

/**
 * @brief Crossover (recombination) operator.
 *
 * We use a BRKGA-style biased uniform crossover:
 * For each gene (i.e., each request index i):
 *   - With probability elite_bias, copy the elite parent's key.
 *   - Otherwise copy the other parent's key.
 *
 * This keeps the genome representation simple and works well with a decoder.
 */
GeneticAlgorithm::Individual GeneticAlgorithm::crossover(const Individual& elite_parent, const Individual& other_parent)
{
	const size_t n = (size_t)instance->request_count;
	std::uniform_real_distribution<float> U(0.0f, 1.0f);

	Individual child;
	child.priority_key.resize(n);
	child.select_key.resize(n);

	for (size_t i = 0; i < n; ++i)
	{
		child.priority_key[i] = (U(rng) < elite_bias) ? elite_parent.priority_key[i] : other_parent.priority_key[i];
		child.select_key[i]   = (U(rng) < elite_bias) ? elite_parent.select_key[i]   : other_parent.select_key[i];
	}
	return child;
}

/**
 * @brief Mutation operator.
 *
 * Mutation introduces randomness to avoid premature convergence.
 * Here we pick @ref mutate_requests random request indices and apply a small
 * gaussian perturbation to both keys, clamped to [0,1].
 */
void GeneticAlgorithm::mutate(Individual& ind)
{
	const size_t n = (size_t)instance->request_count;
	if (n == 0 || mutate_requests <= 0)
		return;

	std::uniform_int_distribution<int> pick_req(0, (int)n - 1);
	std::normal_distribution<float> N(0.0f, mutate_sigma);

	auto clamp01 = [](float x)
	{
		if (x < 0.0f) return 0.0f;
		if (x > 1.0f) return 1.0f;
		return x;
	};

	for (int t = 0; t < mutate_requests; ++t)
	{
		const int i = pick_req(rng);
		ind.priority_key[i] = clamp01(ind.priority_key[i] + N(rng));
		ind.select_key[i]   = clamp01(ind.select_key[i] + N(rng));
	}
};

/**
 * @brief Choose and order which requests this individual will attempt.
 *
 * This function is where we enforce the "serve at least min_requests" rule.
 *
 * Conceptually:
 * - select_key controls the subset of served requests.
 * - priority_key controls the order.
 */
std::vector<int> GeneticAlgorithm::build_selected_requests(const Individual& ind) const
{
	const int n = (int)instance->request_count;
	std::vector<int> all(n);
	std::iota(all.begin(), all.end(), 0);

	// 1) Select requests based on the threshold.
	std::vector<int> selected;
	selected.reserve(n);
	for (int i = 0; i < n; ++i)
	{
		if (ind.select_key[i] < select_threshold)
			selected.push_back(i);
	}

	// 2) Repair: ensure we have at least min_requests.
	const int gamma = instance->min_requests;
	if ((int)selected.size() < gamma)
	{
		// Sort all request indices by select_key so we add "most selected" first.
		std::sort(all.begin(), all.end(),
			[&](int a, int b) { return ind.select_key[a] < ind.select_key[b]; });

		for (int id : all)
		{
			if ((int)selected.size() >= gamma)
				break;
			if (std::find(selected.begin(), selected.end(), id) == selected.end())
				selected.push_back(id);
		}
	}

	// 3) Order the selected set by priority_key.
	std::sort(selected.begin(), selected.end(),
		[&](int a, int b) { return ind.priority_key[a] < ind.priority_key[b]; });

	return selected;
}

/**
 * @brief Decode genome -> Solution (phenotype).
 *
 * This decoder is *feasibility-oriented*:
 * - It never interleaves requests.
 * - It appends pickup then its matching dropoff immediately.
 *
 * Why? Because it is easy to reason about and guarantees:
 * - pickup-before-dropoff
 * - capacity is respected by checking before pickup
 *
 * Downside: it is restrictive, so it may miss good solutions.
 */
Solution GeneticAlgorithm::decode_to_solution(const Individual& ind) const
{
	/**
	 * Temporary minimal vehicle state used during decoding.
	 *
	 * We only need:
	 * - the visited path (list of nodes)
	 * - the current load
	 */
	struct TempDriver
	{
		int id = -1;
		std::vector<Node*> path;
		int load = 0;
	};

	// Create one TempDriver per vehicle.
	std::vector<TempDriver> drivers;
	drivers.reserve(instance->vehicle_count);
	for (int k = 0; k < instance->vehicle_count; ++k)
		drivers.push_back(TempDriver{ k });

	// Determine which requests to serve and in what order.
	const std::vector<int> selected = build_selected_requests(ind);

	// Greedy assignment: for each request in order, put it on the first feasible vehicle.
	for (int rid : selected)
	{
		Node* pickup = &instance->pickups[(size_t)rid];
		Node* dropoff = &instance->dropoffs[(size_t)rid];

		const int pickup_delta = pickup->capacity_change;   // positive
		const int dropoff_delta = dropoff->capacity_change; // negative

		for (auto& d : drivers)
		{
			// Capacity check only needed at pickup.
			if (d.load + pickup_delta > instance->vehicle_capacity)
				continue;

			// Visit pickup.
			d.path.push_back(pickup);
			d.load += pickup_delta;

			// Immediately visit dropoff.
			d.path.push_back(dropoff);
			d.load += dropoff_delta;

			break; // request assigned
		}
	}

	// Convert TempDrivers into a real Solution (one Route per vehicle).
	Solution sol(instance);
	sol.routes.reserve((size_t)instance->vehicle_count);

	for (const auto& d : drivers)
	{
		Route route;
		route.depot = instance->depot;
		route.nodes = { d.path.begin(), d.path.end() };
		route.total_distance = instance->calculate_route_time(route);
		sol.routes.push_back(std::move(route));
	}

	return sol;
}

/**
 * @brief ImGui parameter controls.
 *
 * This lets you tune GA behaviour at runtime.
 */
void GeneticAlgorithm::draw()
{
#ifndef SCFPDP_HEADLESS
	ImGui::SliderInt("Population", &population_size, 20, 600);
	ImGui::SliderFloat("Elite Fraction", &elite_fraction, 0.05f, 0.50f);
	ImGui::SliderFloat("Mutant Fraction", &mutant_fraction, 0.00f, 0.50f);
	ImGui::SliderFloat("Elite Bias", &elite_bias, 0.50f, 0.95f);

	ImGui::SliderInt("Generations/Step", &generations_per_step, 1, 100);
	ImGui::SliderInt("Max Generations", &max_generations, 50, 200000);

	ImGui::SliderFloat("Select Threshold", &select_threshold, 0.05f, 0.95f);
	ImGui::SliderInt("Mutate Requests", &mutate_requests, 0, 20);
	ImGui::SliderFloat("Mutate Sigma", &mutate_sigma, 0.01f, 0.50f);

	ImGui::Separator();
	ImGui::Text("Generation: %zu", generation);
	ImGui::Text("Best objective: %.2f", best_individual.objective);
#endif // SCFPDP_HEADLESS
}
