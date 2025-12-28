#include "instance.h"

#include <cmath>
#include <fstream>
#include <string>
#include <iostream>
#include <stdexcept>
#include <numbers>
#include <cassert>

void Instance::compute_distance_matrix()
{
	for (size_t i = 0; i < nodes.size(); ++i)
	{
		for (size_t j = 0; j < nodes.size(); ++j)
		{
			distance_matrix[i][j] = distance_matrix[j][i] =
				nodes[i].location.distance_to(nodes[j].location);
		}
		depot_distances[i] = depot.distance_to(nodes[i].location);
	}
}

Instance::Instance(int request_count)
	: request_count(request_count), node_count(request_count * 2)
{
	requests = std::span<CustomerRequest>(new CustomerRequest[request_count], request_count);
	nodes = std::span<Node>(new Node[node_count], node_count);
	pickups = std::span<Node>(nodes.data(), request_count);
	dropoffs = std::span<Node>(nodes.data() + request_count, request_count);
	distance_matrix = new dist_t*[node_count];
	for (size_t i = 0; i < node_count; ++i)
		distance_matrix[i] = new dist_t[node_count];
	depot_distances = new dist_t[node_count];
}

Instance::~Instance()
{
	for (size_t i = 0; i < nodes.size(); ++i)
		delete[] distance_matrix[i];
	delete[] distance_matrix;
	delete[] depot_distances;
	delete[] requests.data();
	delete[] nodes.data();
}

Node& Instance::get_pickup_node(const Node* dropoff)
{
	assert(dropoff >= dropoffs.data() &&
		dropoff < dropoffs.data() + request_count);
	return pickups[dropoff - dropoffs.data()];
}

dist_t Instance::get_distance(const Node* a) const
{
	return depot_distances[get_index(a)];
}

dist_t Instance::get_distance(const Node* a, const Node* b) const
{
	return distance_matrix[get_index(a)][get_index(b)];
}

CustomerRequest& Instance::get_request_from_node(const Node* node)
{
	assert(node >= nodes.data() &&
		node < nodes.data() + node_count);
	auto index = get_index(node);
	return requests[
		index < request_count
			? index
			: index - request_count
	];
}

uint64_t Instance::get_index(const Node* node) const
{
	assert(node >= nodes.data() &&
	       node < nodes.data() + node_count);
	return node - nodes.data();
}

uint64_t Instance::get_index(const CustomerRequest* request) const
{
	assert(request >= requests.data() &&
	       request < requests.data() + request_count);
	return request - requests.data();
}

uint64_t Instance::calculate_route_time(const Route& route) const
{
	if (route.nodes.empty())
		return 0;

	uint64_t total = get_distance(route.nodes.front()) + get_distance(route.nodes.back());
	if (route.nodes.size() == 1)
		return total;

	for (auto it = route.nodes.begin(); it != std::prev(route.nodes.end()); ++it)
		total += get_distance(*it, *std::next(it));
	return total;
}

Instance Instance::load_from_file(const char* filename)
{
	std::ifstream file(filename);
	std::string line;
	if (!file.is_open())
		throw std::runtime_error("Could not open file");

	// create instance
	int request_count;
	file >> request_count;
	Instance instance(request_count);

	// read instance attributes
	file >> instance.vehicle_count;
	file >> instance.vehicle_capacity;
	file >> instance.min_requests;
	file >> instance.fairness;
	std::getline(file, line); // go to next line
	std::getline(file, line); // consume the "# demands" line

	// read customer requests from next line
	for (size_t i = 0; i < instance.request_count; ++i)
	{
		instance.requests[i].id = i;
		file >> instance.requests[i].amount;
	}
	std::getline(file, line); // go to next line
	std::getline(file, line); // consume the "# request locations" line

	// read the depot location
	file >> instance.depot.x >> instance.depot.y;

	// read pickup locations
	for (size_t i = 0; i < instance.request_count; ++i)
	{
		Node &node = instance.pickups[i];
		file >> node.location.x >> node.location.y;
		instance.requests[i].pickup_node = &node;
		node.capacity_change = instance.requests[i].amount;
	}
	// read dropoff locations
	for (size_t i = 0; i < instance.request_count; ++i)
	{
		Node &node = instance.dropoffs[i];
		file >> node.location.x >> node.location.y;
		instance.requests[i].dropoff_node = &node;
		node.capacity_change = -instance.requests[i].amount;
	}
	instance.compute_distance_matrix();
	return instance;
}

void Instance::draw(Canvas& canvas, bool draw_links) const
{
	if (draw_links)
	{
		for (const auto& request : requests)
		{
			canvas.add(Line{
				request.pickup_node->location,
				request.dropoff_node->location,
				COLOR_DARK_GRAY
			});
		}
	}
	for (const auto& node : pickups)
		canvas.add(Circle{ static_cast<float>(node.location.x), static_cast<float>(node.location.y), .5f, COLOR_CYAN, true });
	for (const auto& node : dropoffs)
		canvas.add(Circle{ static_cast<float>(node.location.x), static_cast<float>(node.location.y), .5f, COLOR_ORANGE, true });
	canvas.add(Circle{ static_cast<float>(depot.x), static_cast<float>(depot.y), .7f, COLOR_MAGENTA, true });
}

void Instance::print() const
{
	std::cout
		<< "Instance attributes:" << std::endl
		<< "  Number of requests: " << request_count << std::endl
		<< "  Number of vehicles: " << vehicle_count << std::endl
		<< "  Vehicle capacity: " << vehicle_capacity << std::endl
		<< "  Minimum requests per vehicle: " << min_requests << std::endl
		<< "  Fairness factor: " << fairness << std::endl;

	std::cout << "Requests:" << std::endl;
	for (const auto& request : requests)
	{
		std::cout
			<< "  Request " << request.id << ": "
			<< "Amount " << request.amount << ", "
			<< "Pickup (" << request.pickup_node->location.x << ", " << request.pickup_node->location.y << "), "
			<< "Dropoff (" << request.dropoff_node->location.x << ", " << request.dropoff_node->location.y << ")"
			<< std::endl;
	}
}
