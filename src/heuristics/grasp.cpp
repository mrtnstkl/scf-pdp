#include "heuristics/grasp.h"
#include "core/solution.h"
#include "heuristics/localsearch.h"

#ifndef SCFPDP_HEADLESS
#include "imgui.h"
#endif // SCFPDP_HEADLESS

void Grasp::Metrics::draw()
{
#ifndef SCFPDP_HEADLESS
	SolutionMetrics::draw();
	ImGui::Text("Iterations before improvement: %zu", iterations_before_improvement);
#endif // SCFPDP_HEADLESS
}

void Grasp::Metrics::print_head(std::ostream& os) const
{
	SolutionMetrics::print_head(os);
	os << "it_before_imp";
}

void Grasp::Metrics::print(std::ostream& os) const
{
	SolutionMetrics::print(os);
	os << ',' << iterations_before_improvement;
}

Grasp::Grasp(Instance* instance, std::unique_ptr<Algorithm> randomized_construction, std::unique_ptr<LocalSearch> local_search)
	: instance(instance)
	, randomized_construction(std::move(randomized_construction))
	, local_search(std::move(local_search))
	, current_best(Solution(instance))
{
	reset();
}

size_t Grasp::step()
{
	metrics.iterations_before_improvement = 0;
	while (true)
	{
		local_search->set_initial_solution([&]()
		{
			randomized_construction->reset();
			while (!randomized_construction->done())
				randomized_construction->step();
			return randomized_construction->get_current_solution();
		}());

		local_search->reset();
		while (!local_search->done())
			local_search->step();

		const Solution new_solution = local_search->get_current_solution();
		const float new_cost = new_solution.calculate_cost();

		if (new_cost < best_cost)
		{
			printf("GRASP: Found new best solution with cost %.2f (previous %.2f)\n",
				new_cost, best_cost);
			best_cost = new_cost;
			current_best = new_solution;
			current_best.calculate_metrics(metrics);
			break;
		}
		++metrics.iterations_before_improvement;
		if (stopping_criterion.should_stop(new_cost))
		{
			finished = true;
			return metrics.step;
		}
	}
	return ++metrics.step;
}

void Grasp::reset()
{
	finished = false;
	best_cost = std::numeric_limits<float>::max();
	metrics.step = 0;
	metrics.iterations_before_improvement = 0;
	current_best = Solution(instance);
	current_best.calculate_metrics(metrics);
	stopping_criterion.reset();
}

bool Grasp::done()
{
	return finished;
}

Solution Grasp::get_current_solution() { return current_best; }

void Grasp::draw()
{
#ifndef SCFPDP_HEADLESS
	ImGui::SeparatorText("Randomized Construction Heuristic");
	randomized_construction->draw();

	ImGui::SeparatorText("Local Search");
	local_search->draw();

	ImGui::SeparatorText("Stopping Criterion");
	stopping_criterion.draw();
#endif // SCFPDP_HEADLESS
}
