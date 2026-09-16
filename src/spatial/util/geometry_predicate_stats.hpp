#pragma once

#include "duckdb/function/function.hpp"
#include "duckdb/execution/expression_executor.hpp"

#include "duckdb/common/types/geometry.hpp"
#include "duckdb/function/scalar_function.hpp"
#include "duckdb/planner/expression.hpp"
#include "duckdb/planner/expression/bound_function_expression.hpp"
#include "duckdb/storage/statistics/base_statistics.hpp"
#include "duckdb/storage/statistics/geometry_stats.hpp"

namespace duckdb {

//------------------------------------------------------------------------------
// Geometry predicate statistics pruning
//------------------------------------------------------------------------------
// Shared helpers that let the spatial "ST_" predicates participate in zonemap pruning via the scalar-function
// filter_prune callback. When a predicate is evaluated against a constant geometry, the callback reads the constant
// operands bbox from its derived statistics and prunes row groups whose column zonemap can never satisfy the predicate.
//
// Note that in this context FILTER_ALWAYS_FALSE means "no rows pass the filter", which also covers rows where the
// predicate evaluates to NULL.

//! How the bounding boxes of the two operands must relate for a row to possibly match the predicate.
enum class GeometryPredicateBBox : uint8_t {
	//! A match implies bbox(arg0) and bbox(arg1) intersect (symmetric, e.g. ST_Intersects).
	INTERSECTS,
	//! A match implies bbox(arg0) contains bbox(arg1) (e.g. ST_Contains, ST_Covers).
	ARG0_COVERS_ARG1,
	//! A match implies bbox(arg1) contains bbox(arg0) (e.g. ST_Within, ST_CoveredBy).
	ARG1_COVERS_ARG0,
	//! A match implies bbox(arg0) equals bbox(arg1) (e.g. ST_Equals).
	EQUALS,
};

//! How the column zonemap must relate to the constant bounding box for a row to possibly match.
enum class GeometryZonemapCheck : uint8_t {
	//! The column zonemap must intersect the constant bounding box.
	COLUMN_INTERSECTS,
	//! The column zonemap must contain the constant bounding box.
	COLUMN_CONTAINS,
};

//! The resolved operands of a binary geometry predicate: the column operand's statistics and the constant operand bbox.
struct GeometryPredicateOperands {
	//! Which of the first two arguments is the column operand.
	idx_t column_idx;
	//! The column operand's statistics (the per-argument statistics look through any CRS-only cast).
	optional_ptr<const BaseStatistics> column_stats;
	//! The constant operand's bounding box, from its derived statistics.
	GeometryExtent const_extent;
};

//! Does this extent belong to a provably empty (or NULL) geometry? Only an extent never extended by any vertex keeps
//! the initial EMPTY bounds; an unknown or NaN-poisoned extent does not, so the two cases are distinguishable.
inline bool GeometryExtentIsEmpty(const GeometryExtent &extent) {
	return extent.x_min == GeometryExtent::EMPTY_MIN && extent.x_max == GeometryExtent::EMPTY_MAX &&
	       extent.y_min == GeometryExtent::EMPTY_MIN && extent.y_max == GeometryExtent::EMPTY_MAX;
}

//! Identify the operands of a binary geometry predicate:
//! - exactly one of the first two arguments must be a (folded) constant
//! - statistics must be available for both.
//! Returns false if the operands don't match that shape (e.g. a spatial join, or both operands constant).
struct GeometryPredicateBindData : public FunctionData {
	bool has_const = false;
	idx_t column_idx = 0;
	GeometryExtent const_extent = GeometryExtent::Empty();

	unique_ptr<FunctionData> Copy() const override {
		auto copy = make_uniq<GeometryPredicateBindData>();
		copy->has_const = has_const;
		copy->column_idx = column_idx;
		copy->const_extent = const_extent;
		return std::move(copy);
	}
	bool Equals(const FunctionData &other_p) const override {
		auto &other = other_p.Cast<GeometryPredicateBindData>();
		return has_const == other.has_const && column_idx == other.column_idx &&
		       const_extent.x_min == other.const_extent.x_min && const_extent.x_max == other.const_extent.x_max &&
		       const_extent.y_min == other.const_extent.y_min && const_extent.y_max == other.const_extent.y_max;
	}
};

inline bool TryCaptureGeometryPredicateConstant(BindScalarFunctionInput &input, bool symmetric,
                                                GeometryPredicateBindData &bind) {
	auto &arguments = input.GetArguments();
	if (arguments.size() < 2) {
		return false;
	}
	const auto is_const = [](const Expression &expr) { return expr.IsFoldable() && !expr.IsVolatile(); };
	const bool first_const = is_const(*arguments[0]);
	const bool second_const = is_const(*arguments[1]);
	if (first_const == second_const) {
		return false;
	}
	if (first_const) {
		if (!symmetric) {
			return false;
		}
		std::swap(arguments[0], arguments[1]);
	}
	Value constant;
	if (!ExpressionExecutor::TryEvaluateScalar(input.GetClientContext(), *arguments[1], constant)) {
		return false;
	}
	bind.has_const = true;
	bind.column_idx = 0;
	bind.const_extent = GeometryExtent::Empty();
	if (!constant.IsNull() && constant.type().id() == LogicalTypeId::GEOMETRY) {
		const auto &wkb = StringValue::Get(constant);
		Geometry::GetExtent(string_t(wkb), bind.const_extent);
	}
	return true;
}

template <bool SYMMETRIC>
unique_ptr<FunctionData> BindGeometryPredicateOperands(BindScalarFunctionInput &input) {
	auto bind = make_uniq<GeometryPredicateBindData>();
	if (!TryCaptureGeometryPredicateConstant(input, SYMMETRIC, *bind)) {
		return nullptr;
	}
	return std::move(bind);
}

inline bool TryGetGeometryPredicateOperands(const FunctionStatisticsPruneInput &input,
                                            GeometryPredicateOperands &operands) {
	if (!input.bind_data) {
		return false;
	}
	auto bind = dynamic_cast<const GeometryPredicateBindData *>(input.bind_data.get());
	if (!bind || !bind->has_const || bind->column_idx != 0) {
		return false;
	}
	if (input.stats.GetStatsType() != StatisticsType::GEOMETRY_STATS) {
		return false;
	}
	operands.column_idx = 0;
	operands.column_stats = &input.stats;
	operands.const_extent = bind->const_extent;
	return true;
}

//! Resolve the zonemap check given the predicate's bbox semantics and the column operand's index.
inline GeometryZonemapCheck ResolveGeometryZonemapCheck(GeometryPredicateBBox bbox, idx_t column_idx) {
	switch (bbox) {
	case GeometryPredicateBBox::EQUALS:
		// A match implies identical bounding boxes, so the zonemap must contain the constant.
		return GeometryZonemapCheck::COLUMN_CONTAINS;
	case GeometryPredicateBBox::ARG0_COVERS_ARG1:
		// Containment pruning is only possible when the column is the covering operand (arg0).
		return column_idx == 0 ? GeometryZonemapCheck::COLUMN_CONTAINS : GeometryZonemapCheck::COLUMN_INTERSECTS;
	case GeometryPredicateBBox::ARG1_COVERS_ARG0:
		// Containment pruning is only possible when the column is the covering operand (arg1).
		return column_idx == 1 ? GeometryZonemapCheck::COLUMN_CONTAINS : GeometryZonemapCheck::COLUMN_INTERSECTS;
	default:
		// INTERSECTS, and the conservatively safe fallback for any future bbox relation.
		return GeometryZonemapCheck::COLUMN_INTERSECTS;
	}
}

//! Execute the actual pruning given the zonemap check, the constant bounding box and the column statistics.
//! The constant extent must not be empty (the caller handles that case, as it depends on the predicate).
inline FilterPropagateResult ExecuteGeometryPredicatePrune(GeometryZonemapCheck check,
                                                           const GeometryExtent &const_extent,
                                                           const BaseStatistics &stats) {
	if (stats.GetStatsType() != StatisticsType::GEOMETRY_STATS) {
		return FilterPropagateResult::NO_PRUNING_POSSIBLE;
	}
	if (!stats.CanHaveNoNull()) {
		// Only NULL values are possible, and NULL never passes the filter.
		return FilterPropagateResult::FILTER_ALWAYS_FALSE;
	}
	const auto &col_extent = GeometryStats::GetExtent(stats);

	// The containment check is only trustworthy when both bounding boxes are fully finite: an unknown or NaN-poisoned
	// axis would make ContainsXY() fail and wrongly prune, so degrade to the intersection check.
	if (check == GeometryZonemapCheck::COLUMN_CONTAINS && col_extent.HasXY() && const_extent.HasXY()) {
		// Every matching row's bbox contains the constant, so the union (zonemap) must contain it too.
		return col_extent.ContainsXY(const_extent) ? FilterPropagateResult::NO_PRUNING_POSSIBLE
		                                           : FilterPropagateResult::FILTER_ALWAYS_FALSE;
	}

	// Every matching row's bbox intersects the constant, so the union (zonemap) must too.
	// This is safe for unknown or NaN-poisoned extents on either side: their comparisons make IntersectsXY() report an
	// intersection, degrading to no pruning. An empty column extent (all rows NULL or empty geometries) is disjoint
	// from the non-empty constant and prunes.
	return col_extent.IntersectsXY(const_extent) ? FilterPropagateResult::NO_PRUNING_POSSIBLE
	                                             : FilterPropagateResult::FILTER_ALWAYS_FALSE;
}

//! filter_prune callback for the symmetric/asymmetric binary geometry predicates.
template <GeometryPredicateBBox BBOX>
FilterPropagateResult GeometryPredicatePruneCallback(const FunctionStatisticsPruneInput &input) {
	GeometryPredicateOperands operands;
	if (!TryGetGeometryPredicateOperands(input, operands)) {
		return FilterPropagateResult::NO_PRUNING_POSSIBLE;
	}
	if (GeometryExtentIsEmpty(operands.const_extent)) {
		// An empty constant intersects/contains/covers nothing, so those predicates can never be satisfied.
		// ST_Equals is the exception: two empty geometries are equal, and empty geometries are invisible to the zonemap
		// so we cannot prune it.
		return BBOX == GeometryPredicateBBox::EQUALS ? FilterPropagateResult::NO_PRUNING_POSSIBLE
		                                             : FilterPropagateResult::FILTER_ALWAYS_FALSE;
	}
	const auto check = ResolveGeometryZonemapCheck(BBOX, operands.column_idx);
	return ExecuteGeometryPredicatePrune(check, operands.const_extent, *operands.column_stats);
}

} // namespace duckdb
