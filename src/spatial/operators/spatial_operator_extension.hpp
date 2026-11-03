#pragma once

#include "duckdb/common/serializer/deserializer.hpp"
#include "duckdb/planner/operator/logical_extension_operator.hpp"

namespace duckdb {

class DatabaseInstance;

struct SpatialOperators {
	static constexpr const char *EXTENSION_NAME = "duckdb_spatial";
	static void Register(DatabaseInstance &db);
	static unique_ptr<LogicalExtensionOperator> Deserialize(Deserializer &reader);
};

} // namespace duckdb
