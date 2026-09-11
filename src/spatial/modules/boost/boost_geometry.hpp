#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/adapted/boost_variant2.hpp>
#include <boost/variant2/variant.hpp>

#include <vector>

namespace duckdb {

namespace bg = boost::geometry;

using BoostPoint = bg::model::d2::point_xy<double>;
using BoostLinestring = bg::model::linestring<BoostPoint>;
using BoostPolygon = bg::model::polygon<BoostPoint, true, true>;
using BoostMultiPoint = bg::model::multi_point<BoostPoint>;
using BoostMultiLinestring = bg::model::multi_linestring<BoostLinestring>;
using BoostMultiPolygon = bg::model::multi_polygon<BoostPolygon>;

using BoostSingle = boost::variant2::variant<BoostPoint, BoostLinestring, BoostPolygon, BoostMultiPoint,
                                             BoostMultiLinestring, BoostMultiPolygon>;

enum class BoostDimension : uint8_t { POINTLIKE = 0, LINEAR = 1, AREAL = 2 };

inline BoostDimension GetDimension(const BoostSingle &geom) {
	switch (geom.index()) {
	case 0:
	case 3:
		return BoostDimension::POINTLIKE;
	case 1:
	case 4:
		return BoostDimension::LINEAR;
	default:
		return BoostDimension::AREAL;
	}
}

class BoostGeometry {
public:
	BoostGeometry() : parts_(1, BoostSingle {BoostPoint {}}), collection_(false) {
	}
	explicit BoostGeometry(BoostSingle part) : parts_(1, std::move(part)), collection_(false) {
	}
	explicit BoostGeometry(std::vector<BoostSingle> parts)
	    : parts_(std::move(parts)), collection_(true) {
	}

	bool IsCollection() const {
		return collection_;
	}

	const BoostSingle &Single() const {
		return parts_.front();
	}

	const std::vector<BoostSingle> &Parts() const {
		return parts_;
	}

private:
	std::vector<BoostSingle> parts_;
	bool collection_;
};

//----------------------------------------------------------------------------------------------------------------------
// Predicates
//
// bg::within and bg::covered_by do not compile across the full type matrix, and neither does any predicate once a
// GEOMETRYCOLLECTION is involved. bg::relate does compile for every single-geometry pair, so every predicate goes
// through a DE-9IM mask instead of its named algorithm -- which is also how PostGIS defines them.
//----------------------------------------------------------------------------------------------------------------------
enum class BoostPredicate : uint8_t {
	INTERSECTS,
	DISJOINT,
	CONTAINS,
	WITHIN,
	COVERS,
	COVERED_BY,
	TOUCHES,
	CROSSES,
	OVERLAPS,
	EQUALS,
	CONTAINS_PROPERLY,
	WITHIN_PROPERLY,
};

bool EvalPredicate(BoostPredicate pred, const BoostSingle &lhs, const BoostSingle &rhs);
bool EvalPredicate(BoostPredicate pred, const BoostGeometry &lhs, const BoostGeometry &rhs);

} // namespace duckdb
