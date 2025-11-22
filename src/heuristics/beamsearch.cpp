#include "heuristics/beamsearch.h"
#include "beamsearch.h"

float BeamSearch::TreeNode::heuristic_cost() const
{
	// todo: implement an actual heuristic
	return delta_calculator.cost();
}

BeamSearch::TreeNode::TreeNode(Instance* instance)
	: delta_calculator(instance)
{
	for (int i = 0; i < instance->vehicle_count; ++i)
		drivers.emplace_back(i);
	for (Node& pickup : instance->pickups)
		unvisited_pickups.insert(&pickup);
}


BeamSearch::BeamSearch(Instance* instance)
	: instance(instance), beam(), best_node({0.f, instance})
{
	reset();
}

void BeamSearch::reset()
{
	finished = false;
	best_node = {0.f, TreeNode(instance)};
	beam.clear();
	beam.insert(best_node);
}

size_t BeamSearch::step()
{
	if (finished)
		return beam.begin()->second.step;

	std::map<float, TreeNode> new_beam;
	auto add_to_beam = [&](float cost, TreeNode&& node)
	{
		if (new_beam.size() < beam_width) [[unlikely]]
		{
			// beam not full yet
			new_beam.insert({cost, std::move(node)});
		}
		else if (cost < new_beam.rbegin()->first)
		{
			// beam full, but new node is better than worst in beam
			new_beam.erase(std::prev(new_beam.end()));
			new_beam.insert({cost, std::move(node)});
		}
	};

	for (auto& [tree_node_cost, tree_node] : beam)
	{
		tree_node.step++;
		if (tree_node.unvisited_pickups.empty()
			&& tree_node.accepted_requests == tree_node.completed_requests)
		{
			// fully expanded node, mark as finished
			tree_node.finished = true;
			continue;
		}

		// expand the tree node:
		// for each driver, try each possible pickup and dropoff
		for (auto& driver : tree_node.drivers)
		{
			// pickups
			for (Node* pickup : tree_node.unvisited_pickups)
			{
				if (!driver.fits(pickup, *instance))
					continue;
				TreeNode new_node = tree_node;
				new_node.accepted_requests++;
				new_node.unvisited_pickups.erase(pickup);
				new_node.delta_calculator.add_stop(driver.id,
					driver.path.empty() ? nullptr : driver.path.back(), pickup);
				new_node.drivers[driver.id].visit(pickup, *instance);
				add_to_beam(new_node.heuristic_cost(), std::move(new_node));
			}
			// dropoffs
			for (CustomerRequest* request : driver.loaded_requests)
			{
				Node* dropoff = request->dropoff_node;
				TreeNode new_node = tree_node;
				new_node.completed_requests++;
				new_node.drivers[driver.id].loaded_requests.erase(request);
				new_node.delta_calculator.add_stop(driver.id, driver.path.back(), dropoff);
				new_node.drivers[driver.id].visit(dropoff, *instance);
				add_to_beam(new_node.heuristic_cost(), std::move(new_node));
			}
		}
	}

	if (new_beam.empty())
	{
		// no expansions possible
		finished = true;
		return beam.begin()->second.step;
	}
	
	beam = new_beam;
	auto best_in_beam = beam.begin();
	// update best node found so far
	if (best_node.second.accepted_requests != best_node.second.completed_requests
		|| best_node.second.completed_requests < instance->min_requests 
		|| best_in_beam->first < best_node.first)
		best_node = *best_in_beam;
	return best_in_beam->second.step;
}

bool BeamSearch::done()
{
	return finished;
}

Solution BeamSearch::get_current_solution()
{
	Solution solution(instance);
	for (const auto& driver : best_node.second.drivers)
	{
		Route route;
		route.depot = instance->depot;
		route.nodes = {driver.path.begin(), driver.path.end()};
		route.total_distance = solution.calculate_route_time(route);
		solution.routes.push_back(std::move(route));
	}
	solution.calculate_metrics(metrics);
	metrics.step = best_node.second.step;
	return solution;
}

void BeamSearch::draw()
{
	ImGui::SliderInt("Beam Width", &beam_width, 1, 20);
}