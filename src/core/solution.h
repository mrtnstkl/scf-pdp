#pragma once

#include <vector>
#include <list>
#include <ostream>

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

struct Instance;

struct Solution
{
	Instance* instance;
	std::vector<Route> routes;

	Solution(Instance* instance, bool init = false);

	float calculate_fairness() const;

	float calculate_cost() const;

	void calculate_metrics(SolutionMetrics& metrics) const;

	/** 
	 * @brief Validate the solution against instance constraints
	 * 
	 * @param only_structure If true, ignores minimum requests requirement
	 * 
	 * @return true if the solution is valid
	 * @return false if the solution is invalid
	 */
	bool validate(bool only_structure = false) const;

	void print() const;

	/**
	 * @brief Draw the solution onto a canvas
	 * 
	 * @param canvas The canvas to draw on
	 */
	void visualize(Canvas& canvas) const;

	/**
	 * @brief Write the solution to a file
	 * 
	 * @param filename The path to the solution file
	 * @param comment The first line of the solution file
	 */
	void write_to_file(const char* filename, const char* comment = "solution") const;

	/** 
	 * @brief Load a solution from a file
	 * 
	 * @param path The path to the solution file
	 * @param instance The instance the solution belongs to
	 * @return Solution The loaded solution
	 * @throws std::runtime_error if the file could not be opened
	 */
	static Solution load_from_file(const char* path, Instance* instance);
};


struct DeltaCalculator
{
	Instance* instance;
	// sum of all route distances
	uint64_t route_sum;
	// `route_distances` either stores the actual distances or
	// their squares, depending on the fairness measure
	// squares are used for Jain's fairness measure (`FAIRNESS_MEASURE == 0`)
	std::vector<uint64_t> route_distances;

	DeltaCalculator(Instance* instance);
	[[deprecated]] DeltaCalculator(Instance* instance, const Solution& solution);
	DeltaCalculator(const Solution& solution);

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
	std::pair<float, float> cost_with_fairness() const;

	template<typename Container>
	void add_stop(int route_id, Container& container, Container::iterator position, const Node* new_node);

	template<typename Container>
	void remove_stop(int route_id, Container& container, Container::iterator position);
};

template <typename Container>
inline void DeltaCalculator::add_stop(int route_id, Container& container,
	Container::iterator position, const Node* new_node)
{
	if (position == container.end())
	{
		const Node* prev_node = container.empty() ? nullptr : *(std::prev(position));
		add_stop(route_id, prev_node, new_node);
	}
	else
	{
		const Node* prev_node = (position == container.begin()) ? nullptr : *(std::prev(position));
		const Node* next_node = *position;
		add_stop(route_id, prev_node, new_node, next_node);
	}
}

template <typename Container>
inline void DeltaCalculator::remove_stop(
	int route_id, Container& container, Container::iterator position)
{
	if (position == container.end())
		return;
	const Node* old_node = *position;
	const Node* prev_node = (position == container.begin()) ? nullptr : *(std::prev(position));
	const Node* next_node = (std::next(position) == container.end()) ? nullptr : *(std::next(position));
	remove_stop(route_id, prev_node, old_node, next_node);
}
