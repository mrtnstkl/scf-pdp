#pragma once

#include <vector>
#include <list>
#include <ostream>

#include "instance.h"
#include "util/canvas.h"
#include "util/gui.h"

struct Route
{
	Location depot; // the depot, which is always the start and end point of the route
	std::list<Node*> nodes; // the nodes in the route, excluding the depot

	uint64_t total_distance;
};

struct SolutionMetrics : public GuiElement
{
	size_t step;
	size_t route_count;
	float total_cost;
	float fairness;
	bool valid;
	virtual void draw();
	virtual void print_head(std::ostream& os) const;
	virtual void print(std::ostream& os) const;
};

std::ostream& operator<<(std::ostream& os, const SolutionMetrics& metrics);

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
	/**
	 * @brief Add a stop to the end of the route
	 * 
	 * @param route_id 
	 * @param prev_node the node before the new node, or nullptr if inserting at start
	 * @param new_node the new node to insert
	 */
	void add_stop(int route_id, const Node* prev_node, const Node* new_node);
	/**
	 * @brief Add a stop between two nodes
	 * 
	 * @param route_id 
	 * @param prev_node the node before the new node, or nullptr if inserting at start
	 * @param new_node the new node to insert
	 * @param next_node the node after the new node
	 */
	void add_stop(int route_id, const Node* prev_node, const Node* new_node, const Node* next_node);
	/**
	 * @brief Remove a stop from the end of the route
	 * 
	 * @param route_id 
	 * @param prev_node the node before the old node, or nullptr if removing from start
	 * @param old_node the old node to remove
	 */
	void remove_stop(int route_id, const Node* prev_node, const Node* old_node);
	/**
	 * @brief Remove a stop between two nodes
	 * 
	 * @param route_id 
	 * @param prev_node the node before the old node, or nullptr if removing from start
	 * @param old_node the old node to remove
	 * @param next_node the node after the old node
	 */
	void remove_stop(int route_id, const Node* prev_node, const Node* old_node, const Node* next_node);
	/**
	 * @brief Swaps two neighboring stops in a route
	 * 
	 * @param route_id
	 * @param node_prev_a the node before node_a, or nullptr if node_a is at start
	 * @param node_a the first node to swap. must be immediately before node_b
	 * @param node_b the second node to swap. must be immediately after node_a
	 * @param node_after_b the node after node_b, or nullptr if node_b is at end
	 */
	void swap_stops(int route_id, const Node* node_prev_a, const Node* node_a, const Node* node_b, const Node* node_after_b);
	float fairness() const;
	float cost() const;
	float cost(float fairness) const;
};
