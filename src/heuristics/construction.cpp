#include "construction.h"

#include <iostream>
#include <map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <ranges>
#include <random>

#include "imgui.h"

#include "../util/timer.hpp"


void ConstructionHeuristic::Driver::visit(Node* node, Instance& instance)
{
	load += node->capacity_change;
	if (path.empty())
		time = instance.get_distance(node);
	else
		time += instance.get_distance(path.back(), node);
	if (node->is_pickup())
		loaded_requests.insert(&instance.get_request_from_node(node));
	else
		loaded_requests.erase(&instance.get_request_from_node(node));
	path.push_back(node);
}

bool ConstructionHeuristic::Driver::fits(Node* node, Instance& instance) const
{
	if (node->is_pickup())
		return load + node->capacity_change <= instance.vehicle_capacity;
	else
	{
		CustomerRequest& cr = instance.get_request_from_node(node);
		return loaded_requests.contains(&cr);
	}
}

void ConstructionHeuristic::reset()
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

ConstructionHeuristic::ConstructionHeuristic(Instance* instance, const char* name)
	: instance_(instance)
{
	gui.name_ = name;
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

void ConstructionHeuristic::compute_next()
{
	if (done_)
		return;
	step();
	if (record_history_)
		history_.emplace_back(get_current_solution());
	else
		history_ = { get_current_solution() };
	step_++;
}

void ConstructionHeuristic::run_to_completion()
{
	while (!done_)
		compute_next();
}

Solution ConstructionHeuristic::get_current_solution()
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

bool ConstructionHeuristic::done() const
{
	return done_;
}

void ConstructionHeuristic::draw()
{
	if (ImGui::CollapsingHeader(gui.name_))
	{
		if (ImGui::Button("Reset"))
			reset();
		if (!done_)
		{
			if (ImGui::SameLine(), ImGui::Button("Step"))
				compute_next();
			if (ImGui::SameLine(), ImGui::Button("End"))
				run_to_completion();
		}
		ImGui::Text("Step: %zu", step_);
		ImGui::Text("Served Requests: %zu", served_requests_);

		gui.canvas_.draw();

		if (ImGui::Checkbox("Enable History", &record_history_))
		{
			gui.solution_index_ = 0;
			if (!record_history_)
			{
				history_ = { get_current_solution() };
				gui.canvas_.clear();
				Solution(instance_).draw(gui.canvas_);
			}
		}
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
			gui.solution_index_ = step_;
			gui.canvas_.clear();
			history_.back().draw(gui.canvas_);
		}

		Solution& soln = record_history_ ? history_.at(gui.solution_index_) : history_.back();

		ImGui::Text("Current Cost: %.2f", soln.calculate_cost());
		ImGui::Text("Current Fairness: %.4f", soln.calculate_jain_fairness());
	}
}


void RandomConstruction::step()
{
	static thread_local std::mt19937 rng(std::random_device{}());
	if (done_)
		return;
	if (step_ == 0)
	{
		for (Node& pickup : instance_->pickups)
			unvisited_pickups_.insert(&pickup);
		for (size_t i = 0; i < instance_->vehicle_count; ++i)
			busy_drivers_.insert({0, Driver(i)});
	}

	// select the driver that is furthest behind
	Driver driver = std::move(busy_drivers_.begin()->second);
	busy_drivers_.erase(busy_drivers_.begin());

	const bool pickup_possible = !unvisited_pickups_.empty() &&
		accepted_requests_ < instance_->min_requests &&
		(driver.load < instance_->vehicle_capacity);
	const bool dropoff_possible = !driver.loaded_requests.empty();

	if (!pickup_possible && !dropoff_possible)
	{
		finished_drivers_.emplace_back(std::move(driver));
		if (busy_drivers_.empty())
			done_ = true;
		return;
	}
	else if (!dropoff_possible || (pickup_possible && rng() % 2 == 0))
	{
		// must pickup
		std::vector<Node*> random_pickups(unvisited_pickups_.begin(), unvisited_pickups_.end());
		std::ranges::shuffle(random_pickups, rng);
		for (Node* pickup : random_pickups)
		{
			if (!driver.fits(pickup, *instance_))
				continue;
			driver.visit(pickup, *instance_);
			unvisited_pickups_.erase(pickup);
			accepted_requests_++;
			busy_drivers_.insert({driver.time, std::move(driver)});
			return;
		}
	}

	// did not pick anything up, so dropoff
	auto it = driver.loaded_requests.begin();
	std::advance(it, rng() % driver.loaded_requests.size());
	CustomerRequest* request = *it;
	driver.visit(request->dropoff_node, *instance_);
	served_requests_++;
	busy_drivers_.insert({driver.time, std::move(driver)});
}


void DeterministicConstructionHeuristic::step()
{
	if (done_)
		return;

	if (step_ == 0)
	{
		for (Node& pickup : instance_->pickups)
			unvisited_pickups_.insert(&pickup);
		// find the closest pickup locations to the depot
		std::vector<Node*> pickups_near_depot = {unvisited_pickups_.begin(), unvisited_pickups_.end()};
		std::ranges::stable_sort(pickups_near_depot, [&](Node* a, Node* b)
			{ return instance_->get_distance(a) < instance_->get_distance(b); }
		);
		for (size_t i = 0; i < std::min<size_t>(instance_->vehicle_count, pickups_near_depot.size()); ++i)
		{
			Node* pickup = pickups_near_depot[i]; // next closest pickup
			unvisited_pickups_.erase(pickup); // remove from unvisited nodes
			Driver driver;
			driver.id = (int)i;
			driver.visit(pickup, *instance_);
			busy_drivers_.insert({driver.time, driver});
		}
	}
	else if (busy_drivers_.empty())
	{
		done_ = true;
		return;
	}

	// select the driver that is furthest behind
	Driver driver = std::move(busy_drivers_.begin()->second);
	busy_drivers_.erase(busy_drivers_.begin());

	// find the closest node that is either...
	std::pair<dist_t, Node*> closest = {UINT16_MAX, nullptr};
	// ...a dropoff for a loaded request...
	for (CustomerRequest* request : driver.loaded_requests)
	{
		const auto dist = instance_->get_distance(request->dropoff_node, driver.path.back());
		if (dist < closest.first)
			closest = {dist, request->dropoff_node};
	}
	// ...or a pickup of a new one that fits (only if we still need more requests)
	if (accepted_requests_ < instance_->min_requests)
	{
		for (Node* pickup : unvisited_pickups_)
		{
			if (driver.load + pickup->capacity_change > instance_->vehicle_capacity)
				continue; // would exceed capacity
			const auto dist = instance_->get_distance(pickup, driver.path.back());
			if (dist < closest.first)
				closest = {dist, pickup};
		}
	}

	Node* closest_node = closest.second;

	// no more suitible nodes, driver is finished
	if (closest_node == nullptr)
	{
		finished_drivers_.emplace_back(std::move(driver));
		return;
	}

	// add the node to the driver's path
	driver.visit(closest_node, *instance_);
	busy_drivers_.insert({driver.time, std::move(driver)});

	if (closest_node->is_pickup())
		unvisited_pickups_.erase(closest_node);
	else
		served_requests_++;
}


