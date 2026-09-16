#pragma once

namespace duckdb {

class ExtensionLoader;

void RegisterBoostModule(ExtensionLoader &loader);

} // namespace duckdb
