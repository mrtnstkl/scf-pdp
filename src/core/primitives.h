#pragma once

#include <cstdint>
#include <cmath>

using dist_t = uint16_t;

struct Location
{
	uint16_t x, y;
	/**
	 * @brief Calculate the Euclidean distance to another location
	 * 
	 * @return dist_t The distance rounded up to the nearest integer
	 */
	dist_t distance_to(const Location& other) const
	{
		const float dx = x - (float) other.x;
		const float dy = y - (float) other.y;
		return std::ceil(std::sqrt(dx * dx + dy * dy));
	}
};

struct Node
{
	Location location;
	// the change in vehicle capacity when visiting this node
	// positive for pickups, negative for dropoffs
	int capacity_change;

	bool is_pickup() const { return capacity_change > 0; }
};