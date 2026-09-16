#include "spatial/modules/boost/boost_module.hpp"
#include "spatial/modules/boost/boost_geometry.hpp"
#include "spatial/modules/boost/boost_ops.hpp"
#include "spatial/modules/boost/boost_serde.hpp"
#include "spatial/spatial_types.hpp"
#include "spatial/util/function_builder.hpp"
#include "spatial/util/geometry_predicate_stats.hpp"

#include "duckdb/execution/expression_executor.hpp"
#include "duckdb/common/types/geometry.hpp"
#include "duckdb/common/vector_operations/binary_executor.hpp"
#include "duckdb/common/vector_operations/ternary_executor.hpp"
#include "duckdb/common/vector_operations/unary_executor.hpp"
#include "duckdb/function/aggregate_function.hpp"

//======================================================================================================================
// This module replaces the GEOS one. GEOS is LGPL-2.1; Boost.Geometry is BSL-1.0 and already vendored.
//
// Twenty-one functions the GEOS module used to register are NOT registered here. Anyone reaching for one of them
// should find the reason in this list rather than in a "function does not exist" error.
//
// No Boost.Geometry equivalent, and each is a substantial algorithm in its own right:
//
//   ST_MakeValid                GEOS builds it on OverlayNG. bg::correct only fixes ring orientation and closure,
//                               which is a different and much weaker operation.
//   ST_Node                     Needs a noder: segment intersection plus snap-rounding.
//   ST_Polygonize               Needs noding, then ring formation and hole assignment.
//   ST_BuildArea                Sits on top of Polygonize.
//   ST_LineMerge                Graph merge over linework. Self-contained, just unwritten.
//   ST_SimplifyPreserveTopology Douglas-Peucker plus a validity check per candidate removal.
//   ST_ConcaveHull              Delaunay based; Boost.Geometry has no triangulation.
//   ST_ReducePrecision          Snap-rounding without collapsing topology.
//   ST_MaximumInscribedCircle   Branch-and-bound over a distance field.
//   ST_VoronoiDiagram           Boost.Polygon has a Voronoi builder, but on integer coordinates, so it needs
//                               scaling, snapping and clipping to an envelope.
//   ST_MinimumRotatedRectangle  Rotating calipers over bg::convex_hull. Small; just unwritten.
//
// GEOS 3.12's coverage API, which has no counterpart anywhere else:
//
//   ST_CoverageClean, ST_CoverageInvalidEdges(_Agg), ST_CoverageSimplify(_Agg), ST_CoverageUnion(_Agg)
//
// Needs a robust polygon clipper rather than a geometry engine:
//
//   ST_AsMVTGeom                Clipping to a tile box must emit valid polygons. Mapbox wagyu (BSL-1.0) is the
//                               intended route -- it is what ClickHouse uses for the same job. ST_AsMVT itself is
//                               unaffected: it lives in the MVT module and never used GEOS.
//
// Dropped as redundant rather than unimplementable:
//
//   ST_Distance_GEOS, ST_DWithin_GEOS   The sgl module already implements ST_Distance and ST_DWithin over GEOMETRY.
//                                       Keeping a _GEOS-suffixed alias for a build with no GEOS in it would lie.
//
// Three behaviours also differ from GEOS without a function going missing -- mixed-dimension ST_Union/ST_SymDifference
// are refused, Z and M are dropped from returned geometry, and GEOMETRYCOLLECTION arguments are refused by every
// predicate except ST_Intersects and ST_Disjoint. Those are pinned in
// tests/sqllogic/sdb/pg/simple/spatial_boost_divergence.test.
//======================================================================================================================

namespace duckdb {

namespace {

BoostGeometry Deserialize(const string_t &blob) {
	return BoostSerde::Deserialize(blob.GetData(), blob.GetSize());
}

template <BoostPredicate PRED>
void ExecutePredicate(DataChunk &args, ExpressionState &state, Vector &result) {
	BinaryExecutor::Execute<string_t, string_t, bool>(
	    args.data[0], args.data[1], result, args.size(), [&](const string_t &lhs_blob, const string_t &rhs_blob) {
		    return EvalPredicate(PRED, Deserialize(lhs_blob), Deserialize(rhs_blob));
	    });
}

template <BoostPredicate PRED>
constexpr bool IsSymmetricPredicate() {
	return PRED == BoostPredicate::INTERSECTS || PRED == BoostPredicate::DISJOINT || PRED == BoostPredicate::TOUCHES ||
	       PRED == BoostPredicate::CROSSES || PRED == BoostPredicate::OVERLAPS || PRED == BoostPredicate::EQUALS;
}

template <BoostPredicate PRED>
unique_ptr<FunctionData> BindPredicate(BindScalarFunctionInput &input) {
	GeoTypes::PropagateCRS(input);
	return BindGeometryPredicateOperands<IsSymmetricPredicate<PRED>()>(input);
}

template <BoostPredicate PRED, GeometryPredicateBBox PRUNE>
void RegisterPredicate(ExtensionLoader &loader, const char *name, const char *description) {
	FunctionBuilder::RegisterScalar(loader, name, [&](ScalarFunctionBuilder &func) {
		func.AddVariant([&](ScalarFunctionVariantBuilder &variant) {
			variant.AddParameter("geom1", LogicalType::GEOMETRY());
			variant.AddParameter("geom2", LogicalType::GEOMETRY());
			variant.SetReturnType(LogicalType::BOOLEAN);

			variant.SetBind(BindPredicate<PRED>);
			variant.SetFunction(ExecutePredicate<PRED>);
			variant.SetFilterPrune(GeometryPredicatePruneCallback<PRUNE>);
			variant.CanThrowErrors();
		});

		func.SetDescription(description);
		func.SetTag("ext", "spatial");
		func.SetTag("category", "relation");
	});
}

string_t Serialize(Vector &result, const BoostGeometry &geom) {
	const auto size = BoostSerde::GetRequiredSize(geom);
	auto blob = StringVector::EmptyString(result, size);
	BoostSerde::Serialize(geom, blob.GetDataWriteable(), size);
	blob.Finalize();
	return blob;
}

template <BoostOverlay OP>
void ExecuteOverlay(DataChunk &args, ExpressionState &state, Vector &result) {
	BinaryExecutor::Execute<string_t, string_t, string_t>(
	    args.data[0], args.data[1], result, args.size(), [&](const string_t &lhs, const string_t &rhs) {
		    return Serialize(result, Overlay(OP, Deserialize(lhs), Deserialize(rhs)));
	    });
}

template <BoostGeometry (*FUNC)(const BoostGeometry &, const BoostGeometry &)>
void ExecuteBinaryGeometry(DataChunk &args, ExpressionState &state, Vector &result) {
	BinaryExecutor::Execute<string_t, string_t, string_t>(
	    args.data[0], args.data[1], result, args.size(), [&](const string_t &lhs, const string_t &rhs) {
		    return Serialize(result, FUNC(Deserialize(lhs), Deserialize(rhs)));
	    });
}

template <BoostGeometry (*FUNC)(const BoostGeometry &, const BoostGeometry &)>
void ExecuteBinaryGeometryEmptyAsNull(DataChunk &args, ExpressionState &state, Vector &result) {
	BinaryExecutor::Execute<string_t, string_t, string_t>(
	    args.data[0], args.data[1], result, args.size(),
	    [&](const string_t &lhs, const string_t &rhs) -> optional<string_t> {
		    try {
			    return Serialize(result, FUNC(Deserialize(lhs), Deserialize(rhs)));
		    } catch (const boost::geometry::empty_input_exception &) {
			    return {};
		    }
	    });
}

template <BoostGeometry (*FUNC)(const BoostGeometry &)>
void ExecuteUnaryGeometryKeepEmpty(DataChunk &args, ExpressionState &state, Vector &result) {
	UnaryExecutor::Execute<string_t, string_t>(args.data[0], result, args.size(), [&](const string_t &blob) {
		const auto geom = Deserialize(blob);
		if (IsEmptyGeometry(geom)) {
			return StringVector::AddStringOrBlob(result, blob);
		}
		return Serialize(result, FUNC(geom));
	});
}

template <BoostGeometry (*FUNC)(const BoostGeometry &)>
void ExecuteUnaryGeometryCollectionAsNull(DataChunk &args, ExpressionState &state, Vector &result) {
	UnaryExecutor::Execute<string_t, string_t>(args.data[0], result, args.size(),
	                                           [&](const string_t &blob) -> optional<string_t> {
		                                           const auto geom = Deserialize(blob);
		                                           if (geom.IsCollection()) {
			                                           return {};
		                                           }
		                                           return Serialize(result, FUNC(geom));
	                                           });
}

template <BoostGeometry (*FUNC)(const BoostGeometry &)>
void ExecuteUnaryGeometry(DataChunk &args, ExpressionState &state, Vector &result) {
	UnaryExecutor::Execute<string_t, string_t>(args.data[0], result, args.size(), [&](const string_t &blob) {
		return Serialize(result, FUNC(Deserialize(blob)));
	});
}

template <bool (*FUNC)(const BoostGeometry &)>
void ExecuteUnaryBool(DataChunk &args, ExpressionState &state, Vector &result) {
	UnaryExecutor::Execute<string_t, bool>(args.data[0], result, args.size(),
	                                       [&](const string_t &blob) { return FUNC(Deserialize(blob)); });
}

void ExecuteBuffer(DataChunk &args, ExpressionState &state, Vector &result) {
	BinaryExecutor::Execute<string_t, double, string_t>(
	    args.data[0], args.data[1], result, args.size(), [&](const string_t &blob, const double distance) {
		    return Serialize(result, Buffer(Deserialize(blob), distance, 8));
	    });
}

void ExecuteBufferSegments(DataChunk &args, ExpressionState &state, Vector &result) {
	TernaryExecutor::Execute<string_t, double, int32_t, string_t>(
	    args.data[0], args.data[1], args.data[2], result, args.size(),
	    [&](const string_t &blob, const double distance, const int32_t segments) {
		    return Serialize(result, Buffer(Deserialize(blob), distance, segments));
	    });
}

void ExecuteSimplify(DataChunk &args, ExpressionState &state, Vector &result) {
	BinaryExecutor::Execute<string_t, double, string_t>(
	    args.data[0], args.data[1], result, args.size(), [&](const string_t &blob, const double tolerance) {
		    return Serialize(result, Simplify(Deserialize(blob), tolerance));
	    });
}

void RegisterBinaryGeometry(ExtensionLoader &loader, const char *name, const scalar_function_t fn,
                            const char *description) {
	FunctionBuilder::RegisterScalar(loader, name, [&](ScalarFunctionBuilder &func) {
		func.AddVariant([&](ScalarFunctionVariantBuilder &variant) {
			variant.AddParameter("geom1", LogicalType::GEOMETRY());
			variant.AddParameter("geom2", LogicalType::GEOMETRY());
			variant.SetReturnType(LogicalType::GEOMETRY());
			variant.SetBind(GeoTypes::PropagateCRS);
			variant.SetFunction(fn);
			variant.CanThrowErrors();
		});
		func.SetDescription(description);
		func.SetTag("ext", "spatial");
		func.SetTag("category", "construction");
	});
}

void RegisterUnaryGeometry(ExtensionLoader &loader, const char *name, const scalar_function_t fn,
                           const char *description) {
	FunctionBuilder::RegisterScalar(loader, name, [&](ScalarFunctionBuilder &func) {
		func.AddVariant([&](ScalarFunctionVariantBuilder &variant) {
			variant.AddParameter("geom", LogicalType::GEOMETRY());
			variant.SetReturnType(LogicalType::GEOMETRY());
			variant.SetBind(GeoTypes::PropagateCRS);
			variant.SetFunction(fn);
			variant.CanThrowErrors();
		});
		func.SetDescription(description);
		func.SetTag("ext", "spatial");
		func.SetTag("category", "construction");
	});
}

struct BoostAggState {
	BoostGeometry *geom;
};

struct BoostUnaryAggFunction {
	template <class STATE>
	static void Initialize(STATE &state) {
		state.geom = nullptr;
	}

	template <class STATE, class OP>
	static void Combine(const STATE &source, STATE &target, AggregateInputData &) {
		if (!source.geom) {
			return;
		}
		if (!target.geom) {
			target.geom = new BoostGeometry(*source.geom);
			return;
		}
		*target.geom = OP::Merge(*target.geom, *source.geom);
	}

	template <class INPUT_TYPE, class STATE, class OP>
	static void Operation(STATE &state, const INPUT_TYPE &input, AggregateUnaryInput &) {
		auto next = BoostSerde::Deserialize(input.GetData(), input.GetSize());
		if (!state.geom) {
			state.geom = new BoostGeometry(std::move(next));
		} else {
			*state.geom = OP::Merge(*state.geom, next);
		}
	}

	template <class INPUT_TYPE, class STATE, class OP>
	static void ConstantOperation(STATE &state, const INPUT_TYPE &input, AggregateUnaryInput &, idx_t) {
		if (!state.geom) {
			state.geom = new BoostGeometry(BoostSerde::Deserialize(input.GetData(), input.GetSize()));
		}
	}

	template <class T, class STATE>
	static void Finalize(STATE &state, T &target, AggregateFinalizeData &finalize_data) {
		if (!state.geom) {
			finalize_data.ReturnNull();
		} else {
			target = Serialize(finalize_data.result, *state.geom);
		}
	}

	template <class STATE>
	static void Destroy(STATE &state, AggregateInputData &) {
		delete state.geom;
		state.geom = nullptr;
	}

	static bool IgnoreNull() {
		return true;
	}
};

struct ST_Union_Agg_Impl : BoostUnaryAggFunction {
	static BoostGeometry Merge(const BoostGeometry &curr, const BoostGeometry &next) {
		return Overlay(BoostOverlay::UNION, curr, next);
	}
};

struct ST_Intersection_Agg_Impl : BoostUnaryAggFunction {
	static BoostGeometry Merge(const BoostGeometry &curr, const BoostGeometry &next) {
		return Overlay(BoostOverlay::INTERSECTION, curr, next);
	}
};

template <class IMPL>
void RegisterGeometryAggregate(ExtensionLoader &loader, const char *name, const char *description) {
	auto agg = AggregateFunction::UnaryAggregate<BoostAggState, string_t, string_t, IMPL>(LogicalType::GEOMETRY(),
	                                                                                      LogicalType::GEOMETRY());
	agg.SetBindCallback(GeoTypes::PropagateCRS);

	FunctionBuilder::RegisterAggregate(loader, name, [&](AggregateFunctionBuilder &func) {
		func.SetFunction(agg);
		func.CanThrowErrors();
		func.SetDescription(description);
		func.SetTag("ext", "spatial");
		func.SetTag("category", "construction");
	});
}

void RegisterUnaryBool(ExtensionLoader &loader, const char *name, const scalar_function_t fn,
                       const char *description) {
	FunctionBuilder::RegisterScalar(loader, name, [&](ScalarFunctionBuilder &func) {
		func.AddVariant([&](ScalarFunctionVariantBuilder &variant) {
			variant.AddParameter("geom", LogicalType::GEOMETRY());
			variant.SetReturnType(LogicalType::BOOLEAN);
			variant.SetFunction(fn);
			variant.CanThrowErrors();
		});
		func.SetDescription(description);
		func.SetTag("ext", "spatial");
		func.SetTag("category", "property");
	});
}

} // namespace

void RegisterBoostModule(ExtensionLoader &loader) {
	using PR = BoostPredicate;
	using BB = GeometryPredicateBBox;

	RegisterPredicate<PR::INTERSECTS, BB::INTERSECTS>(loader, "ST_Intersects",
	                                                  "Returns true if the geometries share any portion of space");
	RegisterPredicate<PR::DISJOINT, BB::INTERSECTS>(loader, "ST_Disjoint",
	                                                "Returns true if the geometries share no portion of space");
	RegisterPredicate<PR::CONTAINS, BB::ARG0_COVERS_ARG1>(
	    loader, "ST_Contains", "Returns true if the first geometry contains the second geometry");
	RegisterPredicate<PR::WITHIN, BB::ARG1_COVERS_ARG0>(
	    loader, "ST_Within", "Returns true if the first geometry is within the second geometry");
	RegisterPredicate<PR::COVERS, BB::ARG0_COVERS_ARG1>(
	    loader, "ST_Covers", "Returns true if every point of the second geometry lies in the first geometry");
	RegisterPredicate<PR::COVERED_BY, BB::ARG1_COVERS_ARG0>(
	    loader, "ST_CoveredBy", "Returns true if every point of the first geometry lies in the second geometry");
	RegisterPredicate<PR::TOUCHES, BB::INTERSECTS>(
	    loader, "ST_Touches", "Returns true if the geometries meet only along their boundaries");
	RegisterPredicate<PR::CROSSES, BB::INTERSECTS>(loader, "ST_Crosses",
	                                               "Returns true if the geometries cross each other");
	RegisterPredicate<PR::OVERLAPS, BB::INTERSECTS>(
	    loader, "ST_Overlaps",
	    "Returns true if the geometries share interior space without either containing the other");
	RegisterPredicate<PR::EQUALS, BB::EQUALS>(loader, "ST_Equals",
	                                          "Returns true if the geometries cover the same space");
	RegisterPredicate<PR::CONTAINS_PROPERLY, BB::ARG0_COVERS_ARG1>(
	    loader, "ST_ContainsProperly",
	    "Returns true if the second geometry lies in the interior of the first geometry, touching no part of its "
	    "boundary");
	RegisterPredicate<PR::WITHIN_PROPERLY, BB::ARG1_COVERS_ARG0>(
	    loader, "ST_WithinProperly",
	    "Returns true if the first geometry lies in the interior of the second geometry, touching no part of its "
	    "boundary");

	RegisterBinaryGeometry(loader, "ST_Intersection", ExecuteOverlay<BoostOverlay::INTERSECTION>,
	                       "Returns the shared portion of the two geometries");
	RegisterBinaryGeometry(loader, "ST_Union", ExecuteOverlay<BoostOverlay::UNION>,
	                       "Returns the combined area of the two geometries");
	RegisterBinaryGeometry(loader, "ST_Difference", ExecuteOverlay<BoostOverlay::DIFFERENCE>,
	                       "Returns the part of the first geometry that the second does not cover");
	RegisterBinaryGeometry(loader, "ST_SymDifference", ExecuteOverlay<BoostOverlay::SYM_DIFFERENCE>,
	                       "Returns the parts of both geometries that the other does not cover");
	RegisterBinaryGeometry(loader, "ST_ClosestPoint", ExecuteBinaryGeometryEmptyAsNull<ClosestPoint>,
	                       "Returns the point on the first geometry closest to the second");
	RegisterBinaryGeometry(loader, "ST_ShortestLine", ExecuteBinaryGeometryEmptyAsNull<ShortestLine>,
	                       "Returns the shortest line between the two geometries");

	RegisterUnaryGeometry(loader, "ST_ConvexHull", ExecuteUnaryGeometry<ConvexHull>,
	                      "Returns the smallest convex polygon enclosing the geometry");
	RegisterUnaryGeometry(loader, "ST_Envelope", ExecuteUnaryGeometry<Envelope>,
	                      "Returns the bounding rectangle of the geometry");
	RegisterUnaryGeometry(loader, "ST_PointOnSurface", ExecuteUnaryGeometry<PointOnSurface>,
	                      "Returns a point guaranteed to lie on the geometry");
	RegisterUnaryGeometry(loader, "ST_Boundary", ExecuteUnaryGeometryCollectionAsNull<Boundary>,
	                      "Returns the boundary of the geometry");
	RegisterUnaryGeometry(loader, "ST_Normalize", ExecuteUnaryGeometry<Normalize>,
	                      "Returns the geometry with its rings in canonical orientation");
	RegisterUnaryGeometry(loader, "ST_RemoveRepeatedPoints", ExecuteUnaryGeometry<RemoveRepeatedPoints>,
	                      "Returns the geometry with consecutive duplicate vertices removed");

	RegisterUnaryBool(loader, "ST_IsValid", ExecuteUnaryBool<IsValid>,
	                  "Returns true if the geometry is well formed");
	RegisterUnaryBool(loader, "ST_IsSimple", ExecuteUnaryBool<IsSimple>,
	                  "Returns true if the geometry does not intersect itself");
	RegisterUnaryBool(loader, "ST_IsRing", ExecuteUnaryBool<IsRing>,
	                  "Returns true if the geometry is a closed and simple linestring");

	FunctionBuilder::RegisterScalar(loader, "ST_Buffer", [&](ScalarFunctionBuilder &func) {
		func.AddVariant([&](ScalarFunctionVariantBuilder &variant) {
			variant.AddParameter("geom", LogicalType::GEOMETRY());
			variant.AddParameter("distance", LogicalType::DOUBLE);
			variant.SetReturnType(LogicalType::GEOMETRY());
			variant.SetBind(GeoTypes::PropagateCRS);
			variant.SetFunction(ExecuteBuffer);
			variant.CanThrowErrors();
		});
		func.AddVariant([&](ScalarFunctionVariantBuilder &variant) {
			variant.AddParameter("geom", LogicalType::GEOMETRY());
			variant.AddParameter("distance", LogicalType::DOUBLE);
			variant.AddParameter("num_triangles", LogicalType::INTEGER);
			variant.SetReturnType(LogicalType::GEOMETRY());
			variant.SetBind(GeoTypes::PropagateCRS);
			variant.SetFunction(ExecuteBufferSegments);
			variant.CanThrowErrors();
		});
		func.SetDescription("Returns a geometry covering everything within the given distance of the input");
		func.SetTag("ext", "spatial");
		func.SetTag("category", "construction");
	});

	FunctionBuilder::RegisterScalar(loader, "ST_Simplify", [&](ScalarFunctionBuilder &func) {
		func.AddVariant([&](ScalarFunctionVariantBuilder &variant) {
			variant.AddParameter("geom", LogicalType::GEOMETRY());
			variant.AddParameter("tolerance", LogicalType::DOUBLE);
			variant.SetReturnType(LogicalType::GEOMETRY());
			variant.SetBind(GeoTypes::PropagateCRS);
			variant.SetFunction(ExecuteSimplify);
			variant.CanThrowErrors();
		});
		func.SetDescription("Returns a simplified version of the geometry, keeping vertices beyond the tolerance");
		func.SetTag("ext", "spatial");
		func.SetTag("category", "construction");
	});

	RegisterGeometryAggregate<ST_Union_Agg_Impl>(loader, "ST_Union_Agg",
	                                             "Computes the union of a set of input geometries");
	RegisterGeometryAggregate<ST_Union_Agg_Impl>(
	    loader, "ST_MemUnion_Agg",
	    "Computes the union of a set of input geometries, merging them one at a time rather than all at once");
	RegisterGeometryAggregate<ST_Intersection_Agg_Impl>(loader, "ST_Intersection_Agg",
	                                                    "Computes the intersection of a set of input geometries");
}

} // namespace duckdb
