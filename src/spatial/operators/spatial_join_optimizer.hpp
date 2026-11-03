#pragma once

#include "duckdb/optimizer/optimizer_extension.hpp"

namespace duckdb {

class ExtensionLoader;

struct SpatialJoinOptimizer {
	static void Register(ExtensionLoader &loader);
	static void Optimize(OptimizerExtensionInput &input, unique_ptr<LogicalOperator> &plan);
};

} // namespace duckdb
