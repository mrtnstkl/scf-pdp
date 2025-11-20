#pragma once

#include <vector>
#include <span>
#include <cstdint>

using dist_t = uint16_t;

struct Location
{
	uint16_t x, y;
	/**
	 * @brief Calculate the Euclidean distance to another location
	 * 
	 * @return dist_t The distance rounded up to the nearest integer
	 */
	dist_t distance_to(const Location& other) const;
};

struct Node
{
	Location location;
	// the change in vehicle capacity when visiting this node
	// positive for pickups, negative for dropoffs
	int capacity_change;

	bool is_pickup() const { return capacity_change > 0; }
};

struct CustomerRequest
{
	int id;
	Node* pickup_node;
	Node* dropoff_node;
	int amount;
};

struct Instance
{
	size_t request_count;
	size_t node_count;
	int vehicle_count;
	int vehicle_capacity;
	int min_requests;
	float fairness;

	Location depot;
	std::span<CustomerRequest> requests;

	// all locations: first pickups, then deliveries
	// managed by Instance
	std::span<Node> nodes;
	// views into nodes
	std::span<Node> pickups, dropoffs;

	dist_t** distance_matrix;
	dist_t* depot_distances; // distances from depot to each node

	void compute_distance_matrix();

public:

	Instance(int request_count);
	~Instance();

	/** 
	 * @brief Get the corresponding pickup node for a given dropoff node
	 * 
	 * @param dropoff A node that is a dropoff and belongs to this instance 
	 * @return Node& The corresponding pickup node
	 */
	Node& get_pickup_node(const Node* dropoff);

	/** 
	 * @brief Get the distance from the depot to a given node
	 * 
	 * Uses the precomputed depot distances.
	 * 
	 * @param a A node that belongs to this instance
	 * @return dist_t The distance from the depot to the node
	 */
	dist_t get_distance(const Node* a) const;

	/** 
	 * @brief Get the distance between two nodes
	 * 
	 * Uses the precomputed distance matrix.
	 * 
	 * @param a A node that belongs to this instance
	 * @param b A node that belongs to this instance
	 * @return dist_t The distance between the two nodes
	 */
	dist_t get_distance(const Node* a, const Node* b) const;

	/**
	 * @brief Get the customer request associated with a given node
	 * 
	 * @param node A node that belongs to this instance
	 * @return CustomerRequest& The associated customer request
	 */
	CustomerRequest& get_request_from_node(const Node* node);

	/**
	 * @brief Get the index of a node in the instance's node list
	 * 
	 * @param node A node that belongs to this instance
	 * @return uint64_t The index of the node
	 */
	uint64_t get_index(const Node* node) const;

	/**
	 * @brief Load an instance from a file
	 * 
	 * @param filename The path to the instance file
	 * @return Instance The loaded instance
	 */
	static Instance load_from_file(const char* filename);
	
	void print() const;
};
