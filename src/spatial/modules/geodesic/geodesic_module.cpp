#include "spatial/modules/geodesic/geodesic_module.hpp"
#include "spatial/spatial_types.hpp"
#include "spatial/spatial_settings.hpp"
#include "spatial/util/function_builder.hpp"
#include "spatial/geometry/sgl.hpp"
#include "spatial/geometry/geometry_serialization.hpp"

#include "duckdb/common/vector_operations/generic_executor.hpp"
#include "duckdb/logging/logger.hpp"
#include "duckdb/execution/expression_executor.hpp"
#include "duckdb/planner/expression/bound_function_expression.hpp"

#include "geodesic.h"

namespace duckdb {

namespace {

//######################################################################################################################
// Geodesic Functions
//######################################################################################################################

constexpr auto EARTH_A = 6378137;
constexpr auto EARTH_F = 1 / 298.257223563;

//======================================================================================================================
// Bind Data
//======================================================================================================================
struct GeodesicBindData final : FunctionData {

	bool always_xy = false;

	unique_ptr<FunctionData> Copy() const override {
		auto result = make_uniq<GeodesicBindData>();
		result->always_xy = always_xy;
		return std::move(result);
	}

	bool Equals(const FunctionData &other) const override {
		auto &data = other.Cast<GeodesicBindData>();
		return always_xy == data.always_xy;
	}

	static unique_ptr<FunctionData> Bind(BindScalarFunctionInput &input) {
		auto &ctx = input.GetClientContext();
		auto &func = input.GetBoundFunction();

		auto result = make_uniq<GeodesicBindData>();

		bool is_set = false;
		result->always_xy = SpatialSettings::AlwaysXY(ctx, is_set);

		if (!is_set) {
			constexpr auto raw_message =
			    "The '%s' function is sensitive to the coordinate axis order of the input geometry.\n"
			    "The current default for this function is to assume [LATITUDE, LONGITUDE] axis order.\n"
			    "This is expected to change to [LONGITUDE, LATITUDE] in the future.\n "
			    "Please explicitly set the 'geometry_always_xy' setting to avoid unexpected changes in behavior.\n"
			    " * 'SET geometry_always_xy = true' to make this function assume all geometries are [LONGITUDE, "
			    "LATITUDE]\n"
			    " * 'SET geometry_always_xy = false' to keep the current behavior and make this warning go away.";

			auto &logger = Logger::Get(ctx);
			logger.WriteLog("Spatial", LogLevel::LOG_WARNING, StringUtil::Format(raw_message, func.GetName().c_str()));
		}

		return std::move(result);
	}
};

//======================================================================================================================
// Local State
//======================================================================================================================

struct GeodesicLocalState final : FunctionLocalState {

	ArenaAllocator arena;
	GeometryAllocator alloc;
	geod_geodesic geod = {};
	geod_polygon poly = {};
	double accum = 0;

	explicit GeodesicLocalState(ClientContext &context, bool is_line)
	    : arena(BufferAllocator::Get(context)), alloc(arena) {

		// Initialize the geodesic object for earth
		geod_init(&geod, EARTH_A, EARTH_F);
		geod_polygon_init(&poly, is_line ? 1 : 0);
	}

	static unique_ptr<FunctionLocalState> InitPolygon(ExpressionState &state, const BoundFunctionExpression &expr,
	                                                  FunctionData *bind_data) {
		return make_uniq<GeodesicLocalState>(state.GetContext(), false);
	}

	static unique_ptr<FunctionLocalState> InitLine(ExpressionState &state, const BoundFunctionExpression &expr,
	                                               FunctionData *bind_data) {
		return make_uniq<GeodesicLocalState>(state.GetContext(), true);
	}

	static GeodesicLocalState &ResetAndGet(ExpressionState &state) {
		auto &local_state = ExecuteFunctionState::GetFunctionState(state)->Cast<GeodesicLocalState>();
		local_state.arena.Reset();
		return local_state;
	}

	void Deserialize(const string_t &blob, sgl::geometry &geom) {
		Serde::Deserialize(geom, arena, blob.GetDataUnsafe(), blob.GetSize());
	}
};

//======================================================================================================================
// ST_Area_Spheroid
//======================================================================================================================

struct ST_Area_Spheroid {

	//--------------------------------------------------------------------------------------o----------------------------
	// Execute (POLYGON_2D)
	//------------------------------------------------------------------------------------------------------------------

	static void ExecutePolygon(DataChunk &args, ExpressionState &state, Vector &result) {
		D_ASSERT(args.data.size() == 1);

		auto &bdata = state.expr.Cast<BoundFunctionExpression>().BindInfo()->Cast<GeodesicBindData>();

		auto &input = args.data[0];
		auto count = args.size();

		auto &ring_vec = ListVector::GetChild(input);
		auto ring_entries = FlatVector::GetData<list_entry_t>(ring_vec);
		auto &coord_vec = ListVector::GetChild(ring_vec);
		auto &coord_vec_children = StructVector::GetEntries(coord_vec);
		auto x_data = FlatVector::GetData<double>(coord_vec_children[0]);
		auto y_data = FlatVector::GetData<double>(coord_vec_children[1]);

		if (bdata.always_xy) {
			std::swap(x_data, y_data);
		}

		geod_geodesic geod = {};
		geod_init(&geod, EARTH_A, EARTH_F);

		geod_polygon poly = {};
		geod_polygon_init(&poly, 0);

		UnaryExecutor::Execute<list_entry_t, double>(input, result, count, [&](list_entry_t polygon) {
			const auto polygon_offset = polygon.offset;
			const auto polygon_length = polygon.length;

			bool first = true;
			double area = 0;
			for (idx_t ring_idx = polygon_offset; ring_idx < polygon_offset + polygon_length; ring_idx++) {
				const auto ring = ring_entries[ring_idx];
				const auto ring_offset = ring.offset;
				const auto ring_length = ring.length;

				geod_polygon_clear(&poly);
				// Note: the last point is the same as the first point, but geographiclib doesn't know that,
				// so skip it.
				for (idx_t coord_idx = ring_offset; coord_idx < ring_offset + ring_length - 1; coord_idx++) {
					geod_polygon_addpoint(&geod, &poly, x_data[coord_idx], y_data[coord_idx]);
				}
				double ring_area;
				geod_polygon_compute(&geod, &poly, 0, 1, &ring_area, nullptr);

				if (first) {
					// Add outer ring
					area += std::abs(ring_area);
					first = false;
				} else {
					// Subtract holes
					area -= std::abs(ring_area);
				}
			}
			return std::abs(area);
		});

		if (count == 1) {
			result.SetVectorType(VectorType::CONSTANT_VECTOR);
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	// Execute (GEOMETRY)
	//------------------------------------------------------------------------------------------------------------------
	template <bool ALWAYS_XY>
	struct Accumulate {
		static void Operation(void *arg, const sgl::geometry &part) {
			if (part.get_type() != sgl::geometry_type::POLYGON) {
				return;
			}

			auto &sstate = *static_cast<GeodesicLocalState *>(arg);

			// Calculate the area of the polygon
			const auto tail = part.get_last_part();
			auto ring = tail;
			if (!ring) {
				return;
			}

			const auto head = ring->get_next();

			do {
				ring = ring->get_next();

				const auto vertex_count = ring->get_vertex_count();
				if (vertex_count < 4) {
					continue;
				}

				geod_polygon_clear(&sstate.poly);

				// Dont add the last vertex
				for (uint32_t i = 0; i < vertex_count - 1; i++) {
					const auto vertex = ring->get_vertex_xy(i);
					if (ALWAYS_XY) {
						geod_polygon_addpoint(&sstate.geod, &sstate.poly, vertex.y, vertex.x);
					} else {
						geod_polygon_addpoint(&sstate.geod, &sstate.poly, vertex.x, vertex.y);
					}
				}

				double area = 0;
				geod_polygon_compute(&sstate.geod, &sstate.poly, 0, 1, &area, nullptr);

				if (ring == head) {
					sstate.accum += std::abs(area);
				} else {
					sstate.accum -= std::abs(area);
				}
			} while (ring != tail);
		}
	};

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {

		const auto &bdata = state.expr.Cast<BoundFunctionExpression>().BindInfo()->Cast<GeodesicBindData>();
		auto &lstate = GeodesicLocalState::ResetAndGet(state);

		UnaryExecutor::Execute<string_t, double>(args.data[0], result, args.size(), [&](const string_t &input) {
			sgl::geometry geom;
			lstate.Deserialize(input, geom);

			// Reset the state
			lstate.accum = 0;

			// Visit all polygons
			if (bdata.always_xy) {
				sgl::ops::visit_polygon_geometries(geom, &lstate, Accumulate<true>::Operation);
			} else {
				sgl::ops::visit_polygon_geometries(geom, &lstate, Accumulate<false>::Operation);
			}

			return lstate.accum;
		});
	}

	//------------------------------------------------------------------------------------------------------------------
	// Documentation
	//------------------------------------------------------------------------------------------------------------------
	static constexpr auto DESCRIPTION = R"(
    Returns the area of a geometry in meters, using an ellipsoidal model of the earth

    The input geometry is assumed to be in the [EPSG:4326](https://en.wikipedia.org/wiki/World_Geodetic_System) coordinate system (WGS84), with [latitude, longitude] axis order and the area is returned in square meters. This function uses the [GeographicLib](https://geographiclib.sourceforge.io/) library, calculating the area using an ellipsoidal model of the earth. This is a highly accurate method for calculating the area of a polygon taking the curvature of the earth into account, but is also the slowest.

    Returns `0.0` for any geometry that is not a `POLYGON`, `MULTIPOLYGON` or `GEOMETRYCOLLECTION` containing polygon geometries.
	)";

	// TODO: add example
	static constexpr auto EXAMPLE = "";

	//------------------------------------------------------------------------------------------------------------------
	// Register
	//------------------------------------------------------------------------------------------------------------------
	static void Register(ExtensionLoader &loader) {
		FunctionBuilder::RegisterScalar(loader, "ST_Area_Spheroid", [](ScalarFunctionBuilder &func) {
			func.AddVariant([](ScalarFunctionVariantBuilder &variant) {
				variant.AddParameter("geom", LogicalType::GEOMETRY());
				variant.SetReturnType(LogicalType::DOUBLE);

				variant.SetInit(GeodesicLocalState::InitPolygon);
				variant.SetBind(GeodesicBindData::Bind);
				variant.SetFunction(Execute);
				variant.CanThrowErrors();
			});

			func.AddVariant([](ScalarFunctionVariantBuilder &variant) {
				variant.AddParameter("poly", GeoTypes::POLYGON_2D());
				variant.SetReturnType(LogicalType::DOUBLE);
				variant.SetBind(GeodesicBindData::Bind);
				variant.SetFunction(ExecutePolygon);
				variant.CanThrowErrors();
			});

			func.SetExample(EXAMPLE);
			func.SetDescription(DESCRIPTION);

			func.SetTag("ext", "spatial");
			func.SetTag("category", "property");
			func.SetTag("category", "spheroid");
		});
	}
};

//======================================================================================================================
// ST_Perimeter_Spheroid
//======================================================================================================================

struct ST_Perimeter_Spheroid {

	//------------------------------------------------------------------------------------------------------------------
	// Execute (POLYGON_2D)
	//------------------------------------------------------------------------------------------------------------------
	static void ExecutePolygon(DataChunk &args, ExpressionState &state, Vector &result) {
		D_ASSERT(args.data.size() == 1);

		const auto &bdata = state.expr.Cast<BoundFunctionExpression>().BindInfo()->Cast<GeodesicBindData>();

		auto &input = args.data[0];
		auto count = args.size();

		auto &ring_vec = ListVector::GetChild(input);
		auto ring_entries = FlatVector::GetData<list_entry_t>(ring_vec);
		auto &coord_vec = ListVector::GetChild(ring_vec);
		auto &coord_vec_children = StructVector::GetEntries(coord_vec);
		auto x_data = FlatVector::GetData<double>(coord_vec_children[0]);
		auto y_data = FlatVector::GetData<double>(coord_vec_children[1]);

		if (bdata.always_xy) {
			std::swap(x_data, y_data);
		}

		geod_geodesic geod = {};
		geod_init(&geod, EARTH_A, EARTH_F);

		geod_polygon poly = {};
		geod_polygon_init(&poly, 0);

		UnaryExecutor::Execute<list_entry_t, double>(input, result, count, [&](list_entry_t polygon) {
			const auto polygon_offset = polygon.offset;
			const auto polygon_length = polygon.length;
			double perimeter = 0;
			for (idx_t ring_idx = polygon_offset; ring_idx < polygon_offset + polygon_length; ring_idx++) {
				const auto ring = ring_entries[ring_idx];
				const auto ring_offset = ring.offset;
				const auto ring_length = ring.length;

				geod_polygon_clear(&poly);
				// Note: the last point is the same as the first point, but geographiclib doesn't know that,
				// so skip it.
				for (idx_t coord_idx = ring_offset; coord_idx < ring_offset + ring_length - 1; coord_idx++) {
					geod_polygon_addpoint(&geod, &poly, x_data[coord_idx], y_data[coord_idx]);
				}

				double ring_perimeter;
				geod_polygon_compute(&geod, &poly, 0, 1, nullptr, &ring_perimeter);

				perimeter += ring_perimeter;
			}
			return perimeter;
		});

		if (count == 1) {
			result.SetVectorType(VectorType::CONSTANT_VECTOR);
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	// Execute (GEOMETRY)
	//------------------------------------------------------------------------------------------------------------------
	template <bool ALWAYS_XY>
	struct Accumulate {
		static void Operation(void *arg, const sgl::geometry &part) {
			if (part.get_type() != sgl::geometry_type::POLYGON) {
				return;
			}

			auto &sstate = *static_cast<GeodesicLocalState *>(arg);

			// Calculate the perimeter of the polygon
			const auto tail = part.get_last_part();
			auto ring = tail;
			if (!ring) {
				return;
			}
			do {
				ring = ring->get_next();

				const auto vertex_count = ring->get_vertex_count();
				if (vertex_count < 4) {
					continue;
				}

				geod_polygon_clear(&sstate.poly);

				// Dont add the last vertex
				for (uint32_t i = 0; i < vertex_count - 1; i++) {
					const auto vertex = ring->get_vertex_xy(i);
					if (ALWAYS_XY) {
						geod_polygon_addpoint(&sstate.geod, &sstate.poly, vertex.y, vertex.x);
					} else {
						geod_polygon_addpoint(&sstate.geod, &sstate.poly, vertex.x, vertex.y);
					}
				}

				double perimeter = 0;
				geod_polygon_compute(&sstate.geod, &sstate.poly, 0, 1, nullptr, &perimeter);
				// Add the perimeter of the ring
				sstate.accum += perimeter;

			} while (ring != tail);
		}
	};

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {

		auto &lstate = GeodesicLocalState::ResetAndGet(state);
		auto &bdata = state.expr.Cast<BoundFunctionExpression>().BindInfo()->Cast<GeodesicBindData>();

		UnaryExecutor::Execute<string_t, double>(args.data[0], result, args.size(), [&](const string_t &input) {
			sgl::geometry geom;
			lstate.Deserialize(input, geom);

			// Reset the state
			lstate.accum = 0;

			// Visit all polygons
			if (bdata.always_xy) {
				sgl::ops::visit_polygon_geometries(geom, &lstate, Accumulate<true>::Operation);
			} else {
				sgl::ops::visit_polygon_geometries(geom, &lstate, Accumulate<false>::Operation);
			}

			return lstate.accum;
		});
	}

	//------------------------------------------------------------------------------------------------------------------
	// Documentation
	//------------------------------------------------------------------------------------------------------------------
	static constexpr auto DESCRIPTION = R"(
		Returns the length of the perimeter in meters using an ellipsoidal model of the earths surface

		The input geometry is assumed to be in the [EPSG:4326](https://en.wikipedia.org/wiki/World_Geodetic_System) coordinate system (WGS84), with [latitude, longitude] axis order and the length is returned in meters. This function uses the [GeographicLib](https://geographiclib.sourceforge.io/) library, calculating the perimeter using an ellipsoidal model of the earth. This is a highly accurate method for calculating the perimeter of a polygon taking the curvature of the earth into account, but is also the slowest.

		Returns `0.0` for any geometry that is not a `POLYGON`, `MULTIPOLYGON` or `GEOMETRYCOLLECTION` containing polygon geometries.
	)";

	// TODO: add example
	static constexpr auto EXAMPLE = "";

	//------------------------------------------------------------------------------------------------------------------
	// Register
	//------------------------------------------------------------------------------------------------------------------
	static void Register(ExtensionLoader &loader) {
		FunctionBuilder::RegisterScalar(loader, "ST_Perimeter_Spheroid", [](ScalarFunctionBuilder &func) {
			func.AddVariant([](ScalarFunctionVariantBuilder &variant) {
				variant.AddParameter("geom", LogicalType::GEOMETRY());
				variant.SetReturnType(LogicalType::DOUBLE);

				variant.SetInit(GeodesicLocalState::InitPolygon);
				variant.SetBind(GeodesicBindData::Bind);
				variant.SetFunction(Execute);
				variant.CanThrowErrors();
			});

			func.AddVariant([](ScalarFunctionVariantBuilder &variant) {
				variant.AddParameter("poly", GeoTypes::POLYGON_2D());
				variant.SetReturnType(LogicalType::DOUBLE);
				variant.SetBind(GeodesicBindData::Bind);
				variant.SetFunction(ExecutePolygon);
				variant.CanThrowErrors();
			});

			func.SetExample(EXAMPLE);
			func.SetDescription(DESCRIPTION);

			func.SetTag("ext", "spatial");
			func.SetTag("category", "property");
			func.SetTag("category", "spheroid");
		});
	}
};

//======================================================================================================================
// ST_Length_Spheroid
//======================================================================================================================

struct ST_Length_Spheroid {

	//------------------------------------------------------------------------------------------------------------------
	// Execute (LINESTRING)
	//------------------------------------------------------------------------------------------------------------------
	static void ExecuteLineString(DataChunk &args, ExpressionState &state, Vector &result) {
		D_ASSERT(args.data.size() == 1);

		const auto &bdata = state.expr.Cast<BoundFunctionExpression>().BindInfo()->Cast<GeodesicBindData>();

		auto &line_vec = args.data[0];
		auto count = args.size();

		auto &coord_vec = ListVector::GetChild(line_vec);
		auto &coord_vec_children = StructVector::GetEntries(coord_vec);
		auto x_data = FlatVector::GetData<double>(coord_vec_children[0]);
		auto y_data = FlatVector::GetData<double>(coord_vec_children[1]);

		if (bdata.always_xy) {
			std::swap(x_data, y_data);
		}

		geod_geodesic geod = {};
		geod_init(&geod, EARTH_A, EARTH_F);

		geod_polygon poly = {};
		geod_polygon_init(&poly, 1);

		UnaryExecutor::Execute<list_entry_t, double>(line_vec, result, count, [&](list_entry_t line) {
			geod_polygon_clear(&poly);

			const auto offset = line.offset;
			const auto length = line.length;
			// Loop over the segments
			for (idx_t j = offset; j < offset + length; j++) {
				geod_polygon_addpoint(&geod, &poly, x_data[j], y_data[j]);
			}
			double linestring_length;
			geod_polygon_compute(&geod, &poly, 0, 1, &linestring_length, nullptr);
			return linestring_length;
		});

		if (count == 1) {
			result.SetVectorType(VectorType::CONSTANT_VECTOR);
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	// Execute (GEOMETRY)
	//------------------------------------------------------------------------------------------------------------------
	template <bool ALWAYS_XY>
	struct Accumulate {
		static void Operation(void *arg, const sgl::geometry &part) {
			if (part.get_type() != sgl::geometry_type::LINESTRING) {
				return;
			}

			auto &sstate = *static_cast<GeodesicLocalState *>(arg);

			const auto vertex_count = part.get_vertex_count();
			if (vertex_count < 2) {
				return;
			}

			geod_polygon_clear(&sstate.poly);

			for (uint32_t i = 0; i < vertex_count; i++) {
				const auto vertex = part.get_vertex_xy(i);
				if (ALWAYS_XY) {
					geod_polygon_addpoint(&sstate.geod, &sstate.poly, vertex.y, vertex.x);
				} else {
					geod_polygon_addpoint(&sstate.geod, &sstate.poly, vertex.x, vertex.y);
				}
			}

			// Calculate the length of the linestring
			double length = 0;
			geod_polygon_compute(&sstate.geod, &sstate.poly, 0, 1, nullptr, &length);

			sstate.accum += length;
		}
	};

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {

		const auto &bdata = state.expr.Cast<BoundFunctionExpression>().BindInfo()->Cast<GeodesicBindData>();
		auto &lstate = GeodesicLocalState::ResetAndGet(state);

		UnaryExecutor::Execute<string_t, double>(args.data[0], result, args.size(), [&](const string_t &input) {
			sgl::geometry geom;
			lstate.Deserialize(input, geom);

			// Reset the state
			lstate.accum = 0;

			// Visit all linestrings
			if (bdata.always_xy) {
				sgl::ops::visit_linestring_geometries(geom, &lstate, Accumulate<true>::Operation);
			} else {
				sgl::ops::visit_linestring_geometries(geom, &lstate, Accumulate<false>::Operation);
			}

			return lstate.accum;
		});
	}

	//------------------------------------------------------------------------------------------------------------------
	// Documentation
	//------------------------------------------------------------------------------------------------------------------
	static constexpr auto DESCRIPTION = R"(
		Returns the length of the input geometry in meters, using an ellipsoidal model of the earth

		The input geometry is assumed to be in the [EPSG:4326](https://en.wikipedia.org/wiki/World_Geodetic_System) coordinate system (WGS84), with [latitude, longitude] axis order and the length is returned in meters. This function uses the [GeographicLib](https://geographiclib.sourceforge.io/) library, calculating the length using an ellipsoidal model of the earth. This is a highly accurate method for calculating the length of a line geometry taking the curvature of the earth into account, but is also the slowest.

		Returns `0.0` for any geometry that is not a `LINESTRING`, `MULTILINESTRING` or `GEOMETRYCOLLECTION` containing line geometries.
	)";

	// TODO: add example
	static constexpr auto EXAMPLE = "";

	//------------------------------------------------------------------------------------------------------------------
	// Register
	//------------------------------------------------------------------------------------------------------------------
	static void Register(ExtensionLoader &loader) {
		FunctionBuilder::RegisterScalar(loader, "ST_Length_Spheroid", [](ScalarFunctionBuilder &func) {
			func.AddVariant([](ScalarFunctionVariantBuilder &variant) {
				variant.AddParameter("geom", LogicalType::GEOMETRY());
				variant.SetReturnType(LogicalType::DOUBLE);

				variant.SetInit(GeodesicLocalState::InitLine);
				variant.SetBind(GeodesicBindData::Bind);
				variant.SetFunction(Execute);
				variant.CanThrowErrors();
			});

			func.AddVariant([](ScalarFunctionVariantBuilder &variant) {
				variant.AddParameter("line", GeoTypes::LINESTRING_2D());
				variant.SetReturnType(LogicalType::DOUBLE);
				variant.SetBind(GeodesicBindData::Bind);
				variant.SetFunction(ExecuteLineString);
				variant.CanThrowErrors();
			});

			func.SetExample(EXAMPLE);
			func.SetDescription(DESCRIPTION);

			func.SetTag("ext", "spatial");
			func.SetTag("category", "property");
			func.SetTag("category", "spheroid");
		});
	}
};

//======================================================================================================================
// ST_Distance_Spheroid
//======================================================================================================================

struct ST_Distance_Spheroid {

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {
		using POINT_TYPE = StructTypeBinary<double, double>;
		using DISTANCE_TYPE = PrimitiveType<double>;

		geod_geodesic geod = {};
		geod_init(&geod, EARTH_A, EARTH_F);

		const auto &bdata = state.expr.Cast<BoundFunctionExpression>().BindInfo()->Cast<GeodesicBindData>();

		if (bdata.always_xy) {
			GenericExecutor::ExecuteBinary<POINT_TYPE, POINT_TYPE, DISTANCE_TYPE>(
			    args.data[0], args.data[1], result, args.size(), [&](const POINT_TYPE &p1, const POINT_TYPE &p2) {
				    double distance;
				    geod_inverse(&geod, p1.b_val, p1.a_val, p2.b_val, p2.a_val, &distance, nullptr, nullptr);
				    return distance;
			    });
		} else {
			GenericExecutor::ExecuteBinary<POINT_TYPE, POINT_TYPE, DISTANCE_TYPE>(
			    args.data[0], args.data[1], result, args.size(), [&](const POINT_TYPE &p1, const POINT_TYPE &p2) {
				    double distance;
				    geod_inverse(&geod, p1.a_val, p1.b_val, p2.a_val, p2.b_val, &distance, nullptr, nullptr);
				    return distance;
			    });
		}
	}

	static constexpr auto DESCRIPTION = R"(
    Returns the distance between two geometries in meters using an ellipsoidal model of the earths surface

	The input geometry is assumed to be in the [EPSG:4326](https://en.wikipedia.org/wiki/World_Geodetic_System) coordinate system (WGS84), with [latitude, longitude] axis order and the distance limit is expected to be in meters. This function uses the [GeographicLib](https://geographiclib.sourceforge.io/) library to solve the [inverse geodesic problem](https://en.wikipedia.org/wiki/Geodesics_on_an_ellipsoid#Solution_of_the_direct_and_inverse_problems), calculating the distance between two points using an ellipsoidal model of the earth. This is a highly accurate method for calculating the distance between two arbitrary points taking the curvature of the earths surface into account, but is also the slowest.
	)";

	static constexpr auto EXAMPLE = R"(
	-- Note: the coordinates are in WGS84 and [latitude, longitude] axis order
	-- Whats the distance between New York and Amsterdam (JFK and AMS airport)?
	SELECT st_distance_spheroid(
	st_point(40.6446, -73.7797),
	st_point(52.3130, 4.7725)
	);
	----
	5863418.7459356235
	-- Roughly 5863km!
	)";

	static void Register(ExtensionLoader &loader) {
		FunctionBuilder::RegisterScalar(loader, "ST_Distance_Spheroid", [](ScalarFunctionBuilder &func) {
			func.AddVariant([](ScalarFunctionVariantBuilder &variant) {
				variant.AddParameter("p1", GeoTypes::POINT_2D());
				variant.AddParameter("p2", GeoTypes::POINT_2D());
				variant.SetReturnType(LogicalType::DOUBLE);
				variant.SetBind(GeodesicBindData::Bind);

				variant.SetFunction(Execute);
				variant.CanThrowErrors();
			});

			func.SetExample(EXAMPLE);
			func.SetDescription(DESCRIPTION);

			func.SetTag("ext", "spatial");
			func.SetTag("category", "relation");
			func.SetTag("category", "spheroid");
		});
	}
};

//======================================================================================================================
// ST_DWithin_Spheroid
//======================================================================================================================

struct ST_DWithin_Spheroid {

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {
		using POINT_TYPE = StructTypeBinary<double, double>;
		using DISTANCE_TYPE = PrimitiveType<double>;
		using BOOL_TYPE = PrimitiveType<bool>;

		geod_geodesic geod = {};
		geod_init(&geod, EARTH_A, EARTH_F);

		const auto &bdata = state.expr.Cast<BoundFunctionExpression>().BindInfo()->Cast<GeodesicBindData>();

		if (bdata.always_xy) {
			GenericExecutor::ExecuteTernary<POINT_TYPE, POINT_TYPE, DISTANCE_TYPE, BOOL_TYPE>(
			    args.data[0], args.data[1], args.data[2], result, args.size(),
			    [&](const POINT_TYPE &p1, const POINT_TYPE &p2, const DISTANCE_TYPE &limit) {
				    double distance;
				    geod_inverse(&geod, p1.b_val, p1.a_val, p2.b_val, p2.a_val, &distance, nullptr, nullptr);
				    return distance <= limit.val;
			    });
		} else {
			GenericExecutor::ExecuteTernary<POINT_TYPE, POINT_TYPE, DISTANCE_TYPE, BOOL_TYPE>(
			    args.data[0], args.data[1], args.data[2], result, args.size(),
			    [&](const POINT_TYPE &p1, const POINT_TYPE &p2, const DISTANCE_TYPE &limit) {
				    double distance;
				    geod_inverse(&geod, p1.a_val, p1.b_val, p2.a_val, p2.b_val, &distance, nullptr, nullptr);
				    return distance <= limit.val;
			    });
		}
	}

	static constexpr auto DESCRIPTION = R"(
		Returns if two POINT_2D's are within a target distance in meters, using an ellipsoidal model of the earths surface

		The input geometry is assumed to be in the [EPSG:4326](https://en.wikipedia.org/wiki/World_Geodetic_System) coordinate system (WGS84), with [latitude, longitude] axis order and the distance is returned in meters. This function uses the [GeographicLib](https://geographiclib.sourceforge.io/) library to solve the [inverse geodesic problem](https://en.wikipedia.org/wiki/Geodesics_on_an_ellipsoid#Solution_of_the_direct_and_inverse_problems), calculating the distance between two points using an ellipsoidal model of the earth. This is a highly accurate method for calculating the distance between two arbitrary points taking the curvature of the earths surface into account, but is also the slowest.
	)";

	// TODO: add example
	static constexpr auto EXAMPLE = "";

	static void Register(ExtensionLoader &loader) {
		FunctionBuilder::RegisterScalar(loader, "ST_DWithin_Spheroid", [](ScalarFunctionBuilder &func) {
			func.AddVariant([](ScalarFunctionVariantBuilder &variant) {
				variant.AddParameter("p1", GeoTypes::POINT_2D());
				variant.AddParameter("p2", GeoTypes::POINT_2D());
				variant.AddParameter("distance", LogicalType::DOUBLE);
				variant.SetReturnType(LogicalType::BOOLEAN);
				variant.SetBind(GeodesicBindData::Bind);

				variant.SetFunction(Execute);
				variant.CanThrowErrors();
			});

			func.SetExample(EXAMPLE);
			func.SetDescription(DESCRIPTION);

			func.SetTag("ext", "spatial");
			func.SetTag("category", "relation");
			func.SetTag("category", "spheroid");
		});
	}
};

} // namespace

void RegisterGeodesicModule(ExtensionLoader &loader) {
	ST_Area_Spheroid::Register(loader);
	ST_Perimeter_Spheroid::Register(loader);
	ST_Length_Spheroid::Register(loader);
	ST_Distance_Spheroid::Register(loader);
	ST_DWithin_Spheroid::Register(loader);
}

} // namespace duckdb
