#include "heuristics/lns.h"

#include <algorithm>
#include <random>
#include <cassert>
#include <filesystem>

#include "core/instance.h"
#include "core/solution.h"
#include "heuristics/construction.h"
#include "heuristics/lns.h"

#ifndef SCFPDP_HEADLESS
#include "imgui.h"
#endif // SCFPDP_HEADLESS

Lns::PartialSolution::PartialSolution(const Solution& solution)
	: sol(solution), delta(solution)
{
	for (auto& request : solution.instance->requests)
		unhandled_requests.push_back(&request);
	for (auto& route : sol.routes)
	{
		for (Node* node : route.nodes)
		{
			const CustomerRequest* cr = &solution.instance->get_request_from_node(node);
			const auto cr_idx = solution.instance->get_index(cr);
			unhandled_requests[cr_idx] = nullptr;
		}
	}
	auto it = std::partition(unhandled_requests.begin(), unhandled_requests.end(),
		[](const CustomerRequest* r) { return r != nullptr; });
	unhandled_requests.erase(it, unhandled_requests.end());
}

void Lns::DestroyRandomRequests::operator()(PartialSolution& ps)
{
	static thread_local std::mt19937 rng(std::random_device{}());
	std::uniform_real_distribution<float> dist(0.f, 1.f);

	for (int route_id = 0; route_id < ps.sol.routes.size(); ++route_id)
	{
		Route& route = ps.sol.routes[route_id];
		std::vector<Node*> pending_dropoff_removals;
		for (auto it = route.nodes.begin(); it != route.nodes.end(); )
		{
			Node* node = *it;
			CustomerRequest* cr = &ps.sol.instance->get_request_from_node(node);
			if (!node->is_pickup())
			{
				auto dropoff_it = std::find(pending_dropoff_removals.begin(), pending_dropoff_removals.end(), node);
				if (dropoff_it != pending_dropoff_removals.end())
				{
					// remove dropoff
					ps.delta.remove_stop(route_id,
						(dropoff_it == pending_dropoff_removals.begin()) ? nullptr : *(std::prev(dropoff_it)),
						*dropoff_it);
					it = route.nodes.erase(std::find(route.nodes.begin(), route.nodes.end(), *dropoff_it));
					pending_dropoff_removals.erase(dropoff_it);
					ps.unhandled_requests.push_back(cr);
					continue;
				}
			}
			else if (dist(rng) < fraction_to_remove)
			{
				// remove pickup
				ps.delta.remove_stop(route_id,
					(it == route.nodes.begin()) ? nullptr : *(std::prev(it)),
					*it);
				it = route.nodes.erase(it);
				pending_dropoff_removals.push_back(cr->dropoff_node);
				continue;
			}
			++it;
		}
	}
}

void Lns::DestroyRandomRequests::draw()
{
#ifndef SCFPDP_HEADLESS
	ImGui::PushID(this);
	ImGui::Text("Destroy: Random Requests");
	ImGui::SliderFloat("Fraction to Remove", &fraction_to_remove, 0.0f, 1.0f);
	ImGui::PopID();
#endif // SCFPDP_HEADLESS
}

void Lns::DestroyMatchRouteTime::operator()(PartialSolution& ps)
{
	uint64_t shortest_route = std::numeric_limits<uint64_t>::max(); 
	for (auto& route : ps.sol.routes)
		shortest_route = std::min(shortest_route, route.total_distance);
	
	for (int route_id = 0; route_id < ps.sol.routes.size(); ++route_id)
	{
		Route& route = ps.sol.routes[route_id];
		if (route.total_distance <= shortest_route)
			continue;
		while (route.total_distance > shortest_route && !route.nodes.empty())
		{
			const auto dropoff_it = std::prev(route.nodes.end());
			Node* dropoff = *dropoff_it;
			CustomerRequest* cr = &ps.sol.instance->get_request_from_node(dropoff);
			Node* pickup = cr->pickup_node;
			const auto pickup_it = std::prev(std::find(route.nodes.rbegin(), route.nodes.rend(), pickup).base());

			ps.unhandled_requests.push_back(cr);
			const auto old_route_sum = ps.delta.route_sum;

			// remove dropoff
			ps.delta.remove_stop(route_id, *std::prev(dropoff_it), dropoff);
			route.nodes.erase(dropoff_it);
			// remove pickup
			ps.delta.remove_stop(route_id,
				(pickup_it == route.nodes.begin()) ? nullptr : *std::prev(pickup_it), pickup,
				(std::next(pickup_it) == route.nodes.end()) ? nullptr : *std::next(pickup_it));
			route.nodes.erase(pickup_it);

			route.total_distance = old_route_sum - ps.delta.route_sum;
		}
	}
}

void Lns::DestroyMatchRouteTime::draw()
{
#ifndef SCFPDP_HEADLESS
	ImGui::Text("Destroy: Shorten Routes");
#endif // SCFPDP_HEADLESS
}

void Lns::RepairGreedyInsertion::operator()(PartialSolution& ps)
{
	GreedyInsertion rebuilder(ps.sol.instance, ps.sol);
	ps.sol = Algorithm::run(rebuilder);
	ps.delta = DeltaCalculator(ps.sol);
}

void Lns::RepairGreedyInsertion::draw()
{
#ifndef SCFPDP_HEADLESS
	ImGui::Text("Repair: Greedy Insertion");
#endif // SCFPDP_HEADLESS
}

void Lns::RepairGreedyAppend::operator()(PartialSolution& ps)
{
	GreedyNearestAny rebuilder(*ps.sol.instance, ps.sol);
	ps.sol = Algorithm::run(rebuilder);
	ps.delta = DeltaCalculator(ps.sol);
}

void Lns::RepairGreedyAppend::draw()
{
#ifndef SCFPDP_HEADLESS
	ImGui::Text("Repair: Greedy Append");
#endif // SCFPDP_HEADLESS
}

LargeNeighborhoodSearch::ModifierFunction::ModifierFunction(std::unique_ptr<Lns::SolutionModifier> f)
	: func(std::move(f))
{
}

void LargeNeighborhoodSearch::ModifierFunction::reset()
{
	weight = 1.f;
	applications = 0;
	sucesses = 0;
}

void LargeNeighborhoodSearch::ModifierFunction::draw()
{
#ifndef SCFPDP_HEADLESS
	ImGui::PushID(this);
	func->draw();
	ImGui::Text("Weight: %.8f (a=%llu, s=%llu)", weight, applications, sucesses);
	ImGui::PopID();
#endif // SCFPDP_HEADLESS
}

LargeNeighborhoodSearch::ModifierFunction& LargeNeighborhoodSearch::roulette_select(std::vector<ModifierFunction>& functions)
{
	static thread_local std::mt19937 rng(std::random_device{}());
	float total_weight = 0.f;
	for (const auto& func : functions)
		total_weight += func.weight;
	std::uniform_real_distribution<float> dist(0.f, total_weight);
	float r = dist(rng);
	for (auto& func : functions)
	{
		if (r < func.weight)
			return func;
		r -= func.weight;
	}
	return functions.back(); // should not reach here
}


LargeNeighborhoodSearch::LargeNeighborhoodSearch(Instance* instance)
	: instance_(instance),
	  current_solution_(Solution(instance))
{
	// initialize modifier functions
	destroy_functions_.emplace_back(std::make_unique<Lns::DestroyRandomRequests>());
	destroy_functions_.emplace_back(std::make_unique<Lns::DestroyRandomRequests>());
	destroy_functions_.emplace_back(std::make_unique<Lns::DestroyMatchRouteTime>());
	repair_functions_.emplace_back(std::make_unique<Lns::RepairGreedyInsertion>());
	repair_functions_.emplace_back(std::make_unique<Lns::RepairGreedyAppend>());

	reset();
}

size_t LargeNeighborhoodSearch::step()
{
	bool improvement_found = false;
	while (!improvement_found && !finished_)
	{
		// select destroy and repair functions
		ModifierFunction& destroyer = roulette_select(destroy_functions_);
		ModifierFunction& repairer = roulette_select(repair_functions_);

		Lns::PartialSolution new_solution = current_solution_;
		// break current solution
		(*destroyer.func)(new_solution);
		destroyer.applications++;
		// repair solution
		(*repairer.func)(new_solution);
		repairer.applications++;

		improvement_found = new_solution.delta.cost() < current_solution_.delta.cost();

		if (improvement_found)
		{
			destroyer.sucesses++;
			repairer.sucesses++;
			current_solution_ = std::move(new_solution);
			current_solution_.sol.calculate_metrics(metrics);
			metrics.step++;
			printf("LNS: Found new best solution with cost %.2f\n", metrics.total_cost);
		}
		finished_ = stopping_criterion.should_stop(metrics.total_cost);

		// adjust weights
		destroyer.weight = destroyer.weight * (1.f - reaction_factor) +
			(reaction_factor * (destroyer.sucesses / float(destroyer.applications)));
		repairer.weight = repairer.weight * (1.f - reaction_factor) +
			(reaction_factor * (repairer.sucesses / float(repairer.applications)));
	}
	return metrics.step;
}
void LargeNeighborhoodSearch::reset()
{
	Solution initial_soln(instance_, true);
	switch (construction_method)
	{
	case FILE:
		if (std::filesystem::exists("solutions/current_solution.sln"))
			initial_soln = Solution::load_from_file("solutions/current_solution.sln", instance_);
		break;
	case GREEDY_NEAREST_ANY:
		initial_soln = Algorithm::run(GreedyNearestAny(instance_));
		break;
	case GREEDY_RANDOMIZED_CLOSEST:
		initial_soln = Algorithm::run(GreedyRandomizedClosest(instance_));
		break;
	case RANDOM_CONSTRUCTION:
		initial_soln = Algorithm::run(RandomConstruction(instance_));;
		break;
	}

	current_solution_ = Lns::PartialSolution(initial_soln);
	current_solution_.sol.calculate_metrics(metrics);
	stopping_criterion.reset();
	finished_ = false;
	metrics.step = 0;

	for (auto& mf : destroy_functions_)
		mf.reset();
	for (auto& mf : repair_functions_)
		mf.reset();
}
bool LargeNeighborhoodSearch::done()
{
	return finished_;
}
Solution LargeNeighborhoodSearch::get_current_solution()
{
	return current_solution_.sol;
}
void LargeNeighborhoodSearch::draw()
{
#ifndef SCFPDP_HEADLESS
	static const char* construction_method_labels[] = {
		"File", "Greedy Nearest Any", "Greedy Randomized Closest", "Random Construction"
	};
	ImGui::Combo("Initial Construction Method", (int*)&construction_method, construction_method_labels, 4);
	ImGui::SliderFloat("Reaction Factor", &reaction_factor, 0.f, 1.f);

	ImGui::SeparatorText("Destroy Functions");
	for (auto& mf : destroy_functions_)
		mf.draw();
	ImGui::SeparatorText("Repair Functions");
	for (auto& mf : repair_functions_)
		mf.draw();
	ImGui::SeparatorText("Stopping Criterion");
	stopping_criterion.draw();
#endif // SCFPDP_HEADLESS
}
