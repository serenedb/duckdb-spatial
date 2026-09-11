#include "spatial/modules/boost/boost_ops.hpp"

#include <algorithm>
#include <stdexcept>
#include <type_traits>

namespace duckdb {

namespace {

template <class T>
constexpr int DimOfType() {
	if constexpr (std::is_same_v<T, BoostPoint> || std::is_same_v<T, BoostMultiPoint>) {
		return 0;
	} else if constexpr (std::is_same_v<T, BoostLinestring> || std::is_same_v<T, BoostMultiLinestring>) {
		return 1;
	} else {
		return 2;
	}
}

int DimOf(const BoostSingle &geom) {
	return static_cast<int>(GetDimension(geom));
}

[[noreturn]] void RejectCollection() {
	throw std::runtime_error("this function does not support GEOMETRYCOLLECTION arguments");
}

const BoostSingle &Only(const BoostGeometry &geom) {
	if (geom.IsCollection()) {
		RejectCollection();
	}
	return geom.Single();
}

// Boost's overlay output can carry parts with no extent -- a shared polygon edge comes back as the edge plus a
// zero-length segment at its end -- which no other engine reports, so they are dropped before anything sees them.
void Prune(BoostMultiPolygon &multi) {
	multi.erase(std::remove_if(multi.begin(), multi.end(),
	                           [](const BoostPolygon &poly) { return bg::area(poly) == 0; }),
	            multi.end());
}

void Prune(BoostMultiLinestring &multi) {
	multi.erase(std::remove_if(multi.begin(), multi.end(),
	                           [](const BoostLinestring &line) { return line.size() < 2 || bg::length(line) == 0; }),
	            multi.end());
}

void Prune(BoostMultiPoint &multi) {
	std::sort(multi.begin(), multi.end(), [](const BoostPoint &a, const BoostPoint &b) {
		return a.x() != b.x() ? a.x() < b.x() : a.y() < b.y();
	});
	multi.erase(std::unique(multi.begin(), multi.end(),
	                        [](const BoostPoint &a, const BoostPoint &b) { return a.x() == b.x() && a.y() == b.y(); }),
	            multi.end());
}

BoostGeometry Collapse(BoostMultiPolygon &&multi) {
	if (multi.size() == 1) {
		return BoostGeometry(BoostSingle {std::move(multi[0])});
	}
	return BoostGeometry(BoostSingle {std::move(multi)});
}

BoostGeometry Collapse(BoostMultiLinestring &&multi) {
	if (multi.size() == 1) {
		return BoostGeometry(BoostSingle {std::move(multi[0])});
	}
	return BoostGeometry(BoostSingle {std::move(multi)});
}

BoostGeometry Collapse(BoostMultiPoint &&multi) {
	if (multi.size() == 1) {
		return BoostGeometry(BoostSingle {multi[0]});
	}
	return BoostGeometry(BoostSingle {std::move(multi)});
}

//----------------------------------------------------------------------------------------------------------------------
// Overlay
//
// Boost.Geometry only compiles the type pairs an operation is actually defined for, so each operation instantiates
// exactly the combinations its own dimension rule allows and the runtime dispatch picks the matching output type.
//----------------------------------------------------------------------------------------------------------------------
template <int OUT_DIM>
struct OverlayOut;

template <>
struct OverlayOut<0> {
	using type = BoostMultiPoint;
};
template <>
struct OverlayOut<1> {
	using type = BoostMultiLinestring;
};
template <>
struct OverlayOut<2> {
	using type = BoostMultiPolygon;
};

template <int OUT_DIM>
typename OverlayOut<OUT_DIM>::type RunIntersection(const BoostSingle &lhs, const BoostSingle &rhs) {
	typename OverlayOut<OUT_DIM>::type out;
	boost::variant2::visit(
	    [&](const auto &a, const auto &b) {
		    using A = std::decay_t<decltype(a)>;
		    using B = std::decay_t<decltype(b)>;
		    if constexpr (std::min(DimOfType<A>(), DimOfType<B>()) >= OUT_DIM) {
			    bg::intersection(a, b, out);
		    }
	    },
	    lhs, rhs);
	Prune(out);
	return out;
}

template <int OUT_DIM>
typename OverlayOut<OUT_DIM>::type RunSameDim(const BoostOverlay op, const BoostSingle &lhs, const BoostSingle &rhs) {
	typename OverlayOut<OUT_DIM>::type out;
	boost::variant2::visit(
	    [&](const auto &a, const auto &b) {
		    using A = std::decay_t<decltype(a)>;
		    using B = std::decay_t<decltype(b)>;
		    if constexpr (DimOfType<A>() == OUT_DIM && DimOfType<B>() == OUT_DIM) {
			    if (op == BoostOverlay::UNION) {
				    bg::union_(a, b, out);
			    } else {
				    bg::sym_difference(a, b, out);
			    }
		    }
	    },
	    lhs, rhs);
	Prune(out);
	return out;
}

template <int OUT_DIM>
typename OverlayOut<OUT_DIM>::type RunDifference(const BoostSingle &lhs, const BoostSingle &rhs) {
	typename OverlayOut<OUT_DIM>::type out;
	boost::variant2::visit(
	    [&](const auto &a, const auto &b) {
		    using A = std::decay_t<decltype(a)>;
		    using B = std::decay_t<decltype(b)>;
		    if constexpr (DimOfType<A>() == OUT_DIM && DimOfType<B>() >= OUT_DIM) {
			    bg::difference(a, b, out);
		    }
	    },
	    lhs, rhs);
	Prune(out);
	return out;
}

void AppendRings(const BoostPolygon &poly, BoostMultiLinestring &out) {
	const auto ring_to_line = [](const auto &ring) {
		BoostLinestring line;
		for (const auto &vertex : ring) {
			line.push_back(vertex);
		}
		return line;
	};
	if (!poly.outer().empty()) {
		out.push_back(ring_to_line(poly.outer()));
	}
	for (const auto &inner : poly.inners()) {
		out.push_back(ring_to_line(inner));
	}
}

BoostGeometry AsMulti(const BoostSingle &geom) {
	return boost::variant2::visit(
	    [](const auto &g) -> BoostGeometry {
		    using G = std::decay_t<decltype(g)>;
		    if constexpr (std::is_same_v<G, BoostPoint>) {
			    BoostMultiPoint multi;
			    multi.push_back(g);
			    return BoostGeometry(BoostSingle {std::move(multi)});
		    } else {
			    return BoostGeometry(BoostSingle {g});
		    }
	    },
	    geom);
}

} // namespace

BoostGeometry Overlay(const BoostOverlay op, const BoostGeometry &lhs_geom, const BoostGeometry &rhs_geom) {
	const auto &lhs = Only(lhs_geom);
	const auto &rhs = Only(rhs_geom);

	const auto ldim = DimOf(lhs);
	const auto rdim = DimOf(rhs);

	switch (op) {
	case BoostOverlay::INTERSECTION: {
		// Two shapes of the same dimension can still meet in a lower one -- crossing lines share a point, polygons
		// sharing an edge share a line -- so take the highest dimension that actually produced something.
		// A zero-measure result means the shapes only meet in a lower dimension: Boost reports crossing lines as a
		// zero-length linestring rather than as nothing, so measure is the test, not container size.
		const auto top = std::min(ldim, rdim);
		if (top == 2) {
			auto areal = RunIntersection<2>(lhs, rhs);
			if (bg::area(areal) > 0) {
				return Collapse(std::move(areal));
			}
		}
		if (top >= 1) {
			auto linear = RunIntersection<1>(lhs, rhs);
			if (bg::length(linear) > 0) {
				return Collapse(std::move(linear));
			}
		}
		auto pointwise = RunIntersection<0>(lhs, rhs);
		if (!pointwise.empty() || top == 0) {
			return Collapse(std::move(pointwise));
		}
		return top == 2 ? Collapse(BoostMultiPolygon {}) : Collapse(BoostMultiLinestring {});
	}
	case BoostOverlay::DIFFERENCE: {
		if (rdim < ldim) {
			return AsMulti(lhs);
		}
		switch (ldim) {
		case 2:
			return Collapse(RunDifference<2>(lhs, rhs));
		case 1:
			return Collapse(RunDifference<1>(lhs, rhs));
		default:
			return Collapse(RunDifference<0>(lhs, rhs));
		}
	}
	case BoostOverlay::UNION:
	case BoostOverlay::SYM_DIFFERENCE: {
		if (ldim != rdim) {
			throw std::runtime_error(
			    "this function needs both geometries to have the same dimension: mixing points, lines and polygons "
			    "would produce a GEOMETRYCOLLECTION, which the geometry engine cannot build");
		}
		switch (ldim) {
		case 2:
			return Collapse(RunSameDim<2>(op, lhs, rhs));
		case 1:
			return Collapse(RunSameDim<1>(op, lhs, rhs));
		default:
			return Collapse(RunSameDim<0>(op, lhs, rhs));
		}
	}
	default:
		throw std::runtime_error("unknown overlay operation");
	}
}

//----------------------------------------------------------------------------------------------------------------------
// Distance
//----------------------------------------------------------------------------------------------------------------------
double Distance(const BoostGeometry &lhs, const BoostGeometry &rhs) {
	if (lhs.IsCollection() || rhs.IsCollection()) {
		double best = std::numeric_limits<double>::infinity();
		for (const auto &a : lhs.Parts()) {
			for (const auto &b : rhs.Parts()) {
				best = std::min(best, bg::distance(a, b));
			}
		}
		return best;
	}
	return bg::distance(lhs.Single(), rhs.Single());
}

bool DistanceWithin(const BoostGeometry &lhs, const BoostGeometry &rhs, const double distance) {
	return Distance(lhs, rhs) <= distance;
}

namespace {

bg::model::segment<BoostPoint> NearestSegment(const BoostGeometry &lhs_geom, const BoostGeometry &rhs_geom) {
	const auto &lhs = Only(lhs_geom);
	const auto &rhs = Only(rhs_geom);

	bg::model::segment<BoostPoint> seg;
	boost::variant2::visit([&](const auto &a, const auto &b) { bg::closest_points(a, b, seg); }, lhs, rhs);
	return seg;
}

} // namespace

BoostGeometry ClosestPoint(const BoostGeometry &lhs, const BoostGeometry &rhs) {
	const auto seg = NearestSegment(lhs, rhs);
	return BoostGeometry(BoostSingle {seg.first});
}

BoostGeometry ShortestLine(const BoostGeometry &lhs, const BoostGeometry &rhs) {
	const auto seg = NearestSegment(lhs, rhs);
	BoostLinestring line;
	line.push_back(seg.first);
	line.push_back(seg.second);
	return BoostGeometry(BoostSingle {std::move(line)});
}

//----------------------------------------------------------------------------------------------------------------------
// Unary
//----------------------------------------------------------------------------------------------------------------------
BoostGeometry Buffer(const BoostGeometry &geom, const double distance, const int quad_segments) {
	const auto &single = Only(geom);

	// PostGIS counts segments per quarter circle, Boost.Geometry per full circle.
	const auto points_per_circle = static_cast<size_t>(std::max(1, quad_segments) * 4);

	BoostMultiPolygon out;
	boost::variant2::visit(
	    [&](const auto &g) {
		    bg::buffer(g, out, bg::strategy::buffer::distance_symmetric<double>(distance),
		               bg::strategy::buffer::side_straight(), bg::strategy::buffer::join_round(points_per_circle),
		               bg::strategy::buffer::end_round(points_per_circle),
		               bg::strategy::buffer::point_circle(points_per_circle));
	    },
	    single);
	return Collapse(std::move(out));
}

BoostGeometry ConvexHull(const BoostGeometry &geom) {
	BoostPolygon hull;
	if (geom.IsCollection()) {
		BoostMultiPoint all;
		for (const auto &part : geom.Parts()) {
			boost::variant2::visit([&](const auto &g) { bg::for_each_point(g, [&](const BoostPoint &p) { all.push_back(p); }); },
			                       part);
		}
		bg::convex_hull(all, hull);
	} else {
		boost::variant2::visit([&](const auto &g) { bg::convex_hull(g, hull); }, geom.Single());
	}
	return BoostGeometry(BoostSingle {std::move(hull)});
}

BoostGeometry Envelope(const BoostGeometry &geom) {
	bg::model::box<BoostPoint> box;
	bg::assign_inverse(box);

	const auto expand = [&](const BoostSingle &part) {
		bg::model::box<BoostPoint> part_box;
		boost::variant2::visit([&](const auto &g) { bg::envelope(g, part_box); }, part);
		bg::expand(box, part_box);
	};

	if (geom.IsCollection()) {
		for (const auto &part : geom.Parts()) {
			expand(part);
		}
	} else {
		expand(geom.Single());
	}

	const auto xmin = bg::get<bg::min_corner, 0>(box);
	const auto ymin = bg::get<bg::min_corner, 1>(box);
	const auto xmax = bg::get<bg::max_corner, 0>(box);
	const auto ymax = bg::get<bg::max_corner, 1>(box);

	// A degenerate extent collapses to the shape it actually covers, the way PostGIS reports it.
	if (xmin == xmax && ymin == ymax) {
		return BoostGeometry(BoostSingle {BoostPoint {xmin, ymin}});
	}
	if (xmin == xmax || ymin == ymax) {
		BoostLinestring line;
		line.push_back(BoostPoint {xmin, ymin});
		line.push_back(BoostPoint {xmax, ymax});
		return BoostGeometry(BoostSingle {std::move(line)});
	}

	BoostPolygon poly;
	poly.outer().push_back(BoostPoint {xmin, ymin});
	poly.outer().push_back(BoostPoint {xmin, ymax});
	poly.outer().push_back(BoostPoint {xmax, ymax});
	poly.outer().push_back(BoostPoint {xmax, ymin});
	poly.outer().push_back(BoostPoint {xmin, ymin});
	return BoostGeometry(BoostSingle {std::move(poly)});
}

BoostGeometry Simplify(const BoostGeometry &geom, const double tolerance) {
	const auto &single = Only(geom);
	return boost::variant2::visit(
	    [&](const auto &g) -> BoostGeometry {
		    using G = std::decay_t<decltype(g)>;
		    if constexpr (DimOfType<G>() == 0) {
			    return BoostGeometry(BoostSingle {g});
		    } else {
			    G out;
			    bg::simplify(g, out, tolerance);
			    return BoostGeometry(BoostSingle {std::move(out)});
		    }
	    },
	    single);
}

BoostGeometry PointOnSurface(const BoostGeometry &geom) {
	const auto &single = Only(geom);
	BoostPoint point;
	boost::variant2::visit(
	    [&](const auto &g) {
		    using G = std::decay_t<decltype(g)>;
		    if constexpr (DimOfType<G>() == 2) {
			    bg::point_on_surface(g, point);
		    } else if constexpr (std::is_same_v<G, BoostPoint>) {
			    point = g;
		    } else {
			    bool found = false;
			    bg::for_each_point(g, [&](const BoostPoint &p) {
				    if (!found) {
					    point = p;
					    found = true;
				    }
			    });
			    if (!found) {
				    throw std::runtime_error("ST_PointOnSurface: geometry is empty");
			    }
		    }
	    },
	    single);
	return BoostGeometry(BoostSingle {point});
}

BoostGeometry Reverse(const BoostGeometry &geom) {
	const auto &single = Only(geom);
	return boost::variant2::visit(
	    [](const auto &g) -> BoostGeometry {
		    using G = std::decay_t<decltype(g)>;
		    if constexpr (std::is_same_v<G, BoostPoint>) {
			    return BoostGeometry(BoostSingle {g});
		    } else {
			    G out = g;
			    bg::reverse(out);
			    return BoostGeometry(BoostSingle {std::move(out)});
		    }
	    },
	    single);
}

BoostGeometry Boundary(const BoostGeometry &geom) {
	const auto &single = Only(geom);
	return boost::variant2::visit(
	    [](const auto &g) -> BoostGeometry {
		    using G = std::decay_t<decltype(g)>;
		    if constexpr (std::is_same_v<G, BoostPolygon>) {
			    BoostMultiLinestring rings;
			    AppendRings(g, rings);
			    return Collapse(std::move(rings));
		    } else if constexpr (std::is_same_v<G, BoostMultiPolygon>) {
			    BoostMultiLinestring rings;
			    for (const auto &poly : g) {
				    AppendRings(poly, rings);
			    }
			    return Collapse(std::move(rings));
		    } else if constexpr (DimOfType<G>() == 1) {
			    BoostMultiPoint ends;
			    if constexpr (std::is_same_v<G, BoostLinestring>) {
				    if (!g.empty() && !bg::equals(g.front(), g.back())) {
					    ends.push_back(g.front());
					    ends.push_back(g.back());
				    }
			    } else {
				    for (const auto &line : g) {
					    if (!line.empty() && !bg::equals(line.front(), line.back())) {
						    ends.push_back(line.front());
						    ends.push_back(line.back());
					    }
				    }
			    }
			    return BoostGeometry(BoostSingle {std::move(ends)});
		    } else {
			    return BoostGeometry(BoostSingle {BoostMultiPoint {}});
		    }
	    },
	    single);
}

BoostGeometry Normalize(const BoostGeometry &geom) {
	const auto &single = Only(geom);
	return boost::variant2::visit(
	    [](const auto &g) -> BoostGeometry {
		    using G = std::decay_t<decltype(g)>;
		    if constexpr (std::is_same_v<G, BoostPoint>) {
			    return BoostGeometry(BoostSingle {g});
		    } else {
			    G out = g;
			    bg::correct(out);
			    return BoostGeometry(BoostSingle {std::move(out)});
		    }
	    },
	    single);
}

BoostGeometry RemoveRepeatedPoints(const BoostGeometry &geom) {
	const auto &single = Only(geom);
	return boost::variant2::visit(
	    [](const auto &g) -> BoostGeometry {
		    using G = std::decay_t<decltype(g)>;
		    if constexpr (std::is_same_v<G, BoostPoint>) {
			    return BoostGeometry(BoostSingle {g});
		    } else {
			    G out = g;
			    bg::unique(out);
			    return BoostGeometry(BoostSingle {std::move(out)});
		    }
	    },
	    single);
}

bool IsValid(const BoostGeometry &geom) {
	if (geom.IsCollection()) {
		for (const auto &part : geom.Parts()) {
			if (!boost::variant2::visit([](const auto &g) { return bg::is_valid(g); }, part)) {
				return false;
			}
		}
		return true;
	}
	return boost::variant2::visit([](const auto &g) { return bg::is_valid(g); }, geom.Single());
}

bool IsSimple(const BoostGeometry &geom) {
	if (geom.IsCollection()) {
		for (const auto &part : geom.Parts()) {
			if (!boost::variant2::visit([](const auto &g) { return bg::is_simple(g); }, part)) {
				return false;
			}
		}
		return true;
	}
	return boost::variant2::visit([](const auto &g) { return bg::is_simple(g); }, geom.Single());
}

bool IsRing(const BoostGeometry &geom) {
	const auto &single = Only(geom);
	if (!boost::variant2::holds_alternative<BoostLinestring>(single)) {
		return false;
	}
	const auto &line = boost::variant2::get<BoostLinestring>(single);
	if (line.size() < 4 || !bg::equals(line.front(), line.back())) {
		return false;
	}
	return bg::is_simple(line);
}

} // namespace duckdb
