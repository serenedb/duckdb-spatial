#pragma once

#include "spatial/modules/boost/boost_geometry.hpp"

#include <cstddef>

namespace duckdb {

struct BoostSerde {
	// Boost.Geometry is strictly 2D: Z and M are dropped on read and never written back.
	static BoostGeometry Deserialize(const char *buffer, size_t buffer_size);
	static size_t GetRequiredSize(const BoostGeometry &geom);
	static void Serialize(const BoostGeometry &geom, char *buffer, size_t buffer_size);
};

} // namespace duckdb
