#include "construction.h"

#include <iostream>
#include <map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <ranges>

#include "imgui.h"

#include "../util/timer.hpp"

void DeterministicConstruction::reset()
{
	Solution empty_solution(instance_);
	gui.canvas_.clear();
	empty_solution.draw(gui.canvas_);
	history_ = {empty_solution};
	step_ = 0;
	done_ = false;
	accepted_requests_ = 0;
	served_requests_ = 0;
	busy_drivers_.clear();
	finished_drivers_.clear();
	unvisited_pickups_.clear();
	gui.solution_index_ = 0;
}

DeterministicConstruction::DeterministicConstruction(Instance* instance, bool record_history)
	: instance_(instance), record_history_(record_history)
{
	const float width_target = 560.f;
	Solution(instance_).draw(gui.canvas_);
	auto [max_x, max_y] = gui.canvas_.get_max_extent();
	gui.canvas_.scale = width_target / max_x;
	gui.canvas_.width = width_target;
	gui.canvas_.height = max_y * gui.canvas_.scale;
	gui.canvas_.bg_color = COLOR_BLACK;
	gui.canvas_.stroke_width = .2f;
	reset();
}

void DeterministicConstruction::step()
{
	if (done_)
		return;
	if (step_ == 0)
	{
		for (Node& pickup : instance_->pickups)
			unvisited_pickups_.insert(&pickup);
		// find the closest pickup locations to the depot
		std::vector<Node*> pickups_near_depot = {unvisited_pickups_.begin(), unvisited_pickups_.end()};
		//std::ranges::stable_sort(pickups_near_depot, [&](Node* a, Node* b)
		//	{ return instance.get_distance(a) > instance.get_distance(b); }
		//);
		for (size_t i = 0; i < std::min<size_t>(instance_->vehicle_count, pickups_near_depot.size()); ++i)
		{
			Node* pickup = pickups_near_depot[i]; // next closest pickup
			unvisited_pickups_.erase(pickup); // remove from unvisited nodes
			Driver driver;
			driver.id = (int)i;
			driver.path.push_back(pickup); // add pickup to driver path
			driver.loaded_requests.insert(&instance_->get_request_from_node(pickup)); // add request to driver
			driver.time = instance_->get_distance(pickup);
			driver.load = pickup->capacity_change; 
			busy_drivers_.insert({driver.time, driver});
		}
	}
	if (busy_drivers_.empty())
	{
		done_ = true;
		return;
	}
	
	// select the driver that is furthest behind
	Driver driver = std::move(busy_drivers_.begin()->second);
	busy_drivers_.erase(busy_drivers_.begin());

	// find the closest node that is either...
	Node* closest_node = nullptr;
	uint16_t closest_dist = UINT16_MAX;
	// ...a dropoff for a loaded request...
	for (CustomerRequest* request : driver.loaded_requests)
	{
		const auto dist = instance_->get_distance(request->dropoff_node, driver.path.back());
		if (dist < closest_dist)
		{
			closest_dist = dist;
			closest_node = request->dropoff_node;
		}
	}
	// ...or a pickup of a new one that fits (only if we still need more requests)
	if (accepted_requests_ < instance_->min_requests)
	{
		for (Node* pickup : unvisited_pickups_)
		{
			if (driver.load + pickup->capacity_change > instance_->vehicle_capacity)
				continue; // would exceed capacity
			const auto dist = instance_->get_distance(pickup, driver.path.back());
			if (dist < closest_dist)
			{
				closest_dist = dist;
				closest_node = pickup;
			}
		}
	}

	// no more suitible nodes, driver is finished
	if (closest_node == nullptr)
	{
		finished_drivers_.emplace_back(std::move(driver));
		return;
	}

	// add the node to the driver's path
	driver.time += closest_dist;
	driver.load += closest_node->capacity_change;
	driver.path.push_back(closest_node);
	if (closest_node->is_pickup())
	{
		unvisited_pickups_.erase(closest_node);
		driver.loaded_requests.insert(&instance_->get_request_from_node(closest_node));
	}
	else
	{
		driver.loaded_requests.erase(&instance_->get_request_from_node(closest_node));
		served_requests_++;
	}
	busy_drivers_.insert({driver.time, std::move(driver)});

	if (record_history_)
		history_.emplace_back(get_current_solution());
	step_++;
}

void DeterministicConstruction::run_to_completion()
{
	while (!done_)
		step();
}

Solution DeterministicConstruction::get_current_solution()
{
	Solution sol(instance_, true);
	for (const auto& [time, driver] : busy_drivers_)
	{
		Route& route = sol.routes.at(driver.id);
		route.nodes = {driver.path.begin(), driver.path.end()};
		route.total_distance = sol.calculate_route_time(route);
	}
	for (auto& driver : finished_drivers_)
	{
		Route& route = sol.routes.at(driver.id);
		route.nodes = {driver.path.begin(), driver.path.end()};
		route.total_distance = sol.calculate_route_time(route);
	}
	return sol;
}

bool DeterministicConstruction::done() const
{
	return done_;
}

void DeterministicConstruction::draw()
{
	if (ImGui::CollapsingHeader("Deterministic Constructor"))
	{
		if (ImGui::Button("Reset"))
			reset();
		if (!done_)
		{
			if (ImGui::SameLine(), ImGui::Button("Step"))
				step();
			if (ImGui::SameLine(), ImGui::Button("End"))
				run_to_completion();
		}
		ImGui::Text("Step: %zu", step_);
		ImGui::Text("Served Requests: %zu", served_requests_);

		gui.canvas_.draw();

		if (record_history_)
		{
			if (ImGui::SliderInt("History", &gui.solution_index_, 0, (int)history_.size() - 1))
			{
				gui.canvas_.clear();
				history_[gui.solution_index_].draw(gui.canvas_);
			}
		}
		else if (gui.solution_index_ != step_)
		{
			gui.canvas_.clear();
			history_ = { get_current_solution() };
			history_.back().draw(gui.canvas_);
			gui.solution_index_ = step_;
		}

		Solution& soln = record_history_ ? history_.at(gui.solution_index_) : history_.back();

		ImGui::Text("Current Cost: %.2f", soln.calculate_cost());
		ImGui::Text("Current Fairness: %.4f", soln.calculate_jain_fairness());
	}
}
