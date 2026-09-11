#pragma once

#include "spatial/modules/boost/boost_geometry.hpp"

namespace duckdb {

enum class BoostOverlay : uint8_t { INTERSECTION, UNION, DIFFERENCE, SYM_DIFFERENCE };

BoostGeometry Overlay(BoostOverlay op, const BoostGeometry &lhs, const BoostGeometry &rhs);

double Distance(const BoostGeometry &lhs, const BoostGeometry &rhs);
bool DistanceWithin(const BoostGeometry &lhs, const BoostGeometry &rhs, double distance);
BoostGeometry ClosestPoint(const BoostGeometry &lhs, const BoostGeometry &rhs);
BoostGeometry ShortestLine(const BoostGeometry &lhs, const BoostGeometry &rhs);

BoostGeometry Buffer(const BoostGeometry &geom, double distance, int quad_segments);
BoostGeometry ConvexHull(const BoostGeometry &geom);
BoostGeometry Envelope(const BoostGeometry &geom);
BoostGeometry Simplify(const BoostGeometry &geom, double tolerance);
BoostGeometry PointOnSurface(const BoostGeometry &geom);
BoostGeometry Reverse(const BoostGeometry &geom);
BoostGeometry Boundary(const BoostGeometry &geom);
BoostGeometry Normalize(const BoostGeometry &geom);
BoostGeometry RemoveRepeatedPoints(const BoostGeometry &geom);

bool IsValid(const BoostGeometry &geom);
bool IsSimple(const BoostGeometry &geom);
bool IsRing(const BoostGeometry &geom);

} // namespace duckdb
