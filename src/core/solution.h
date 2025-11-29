#pragma once

#include <vector>
#include <list>

#include "instance.h"
#include "util/canvas.h"

struct Route
{
	Location depot; // the depot, which is always the start and end point of the route
	std::list<Node*> nodes; // the nodes in the route, excluding the depot

	uint64_t total_distance;
};

struct SolutionMetrics
{
	size_t step;
	size_t route_count;
	float total_cost;
	float fairness;
	bool valid;
};


struct Solution
{
	Instance* instance;
	std::vector<Route> routes;

	Solution(Instance* instance, bool init = false);

	/** 
	 * @brief Calculate the total time (distance) of a given route
	 * 
	 * Uses the instance's distance matrix.
	 * TODO: maybe move this to Instance?
	 * 
	 * @param route The route to calculate the time for
	 * @return int The total time (distance) of the route
	 */
	uint64_t calculate_route_time(const Route& route) const;

	float calculate_jain_fairness() const;

	float calculate_cost() const;

	void calculate_metrics(SolutionMetrics& metrics) const;

	/** 
	 * @brief Validate the solution against instance constraints
	 * 
	 * @return true if the solution is valid
	 * @return false if the solution is invalid
	 */
	bool validate() const;

	void print() const;

	void draw(Canvas& canvas) const;

	void write_to_file(const char* filename, const char* comment = "solution") const;

	static Solution load_from_file(const char* path, Instance* instance);
};


struct DeltaCalculator
{
	Instance* instance;
	uint64_t route_sum;
	std::vector<uint64_t> route_squares;

	DeltaCalculator(Instance* instance);
	DeltaCalculator(Instance* instance, const Solution& solution);
	void add_stop(int route_id, const Node* prev_node, const Node* new_node);
	void add_stop(int route_id, const Node* prev_node, const Node* new_node, const Node* next_node);
	void remove_stop(int route_id, const Node* prev_node, const Node* old_node);
	void remove_stop(int route_id, const Node* prev_node, const Node* old_node, const Node* next_node);
	float fairness() const;
	float cost() const;
	float cost(float fairness) const;
};


#include "imgui.h"
template<>
inline void draw<SolutionMetrics>(SolutionMetrics& drawable)
{
	ImGui::Text("Route Count: %zu", drawable.route_count);
	ImGui::Text("Total Cost: %.2f", drawable.total_cost);
	ImGui::Text("Fairness: %.4f", drawable.fairness);
	ImGui::Text("Valid: %s", drawable.valid ? "Yes" : "No");
}
