#include "spatial/modules/boost/boost_geometry.hpp"

#include <stdexcept>

namespace duckdb {

namespace {

auto CoveredByMask() {
	return bg::de9im::mask("T*F**F***") || bg::de9im::mask("*TF**F***") || bg::de9im::mask("**FT*F***") ||
	       bg::de9im::mask("**F*TF***");
}

bool EvalSingle(BoostPredicate pred, const BoostSingle &lhs, const BoostSingle &rhs) {
	switch (pred) {
	case BoostPredicate::INTERSECTS:
		return bg::intersects(lhs, rhs);
	case BoostPredicate::DISJOINT:
		return bg::disjoint(lhs, rhs);
	case BoostPredicate::TOUCHES:
		return bg::touches(lhs, rhs);
	case BoostPredicate::CROSSES:
		return bg::crosses(lhs, rhs);
	case BoostPredicate::OVERLAPS:
		return bg::overlaps(lhs, rhs);
	case BoostPredicate::EQUALS:
		return bg::equals(lhs, rhs);
	case BoostPredicate::WITHIN:
		return bg::relate(lhs, rhs, bg::de9im::mask("T*F**F***"));
	case BoostPredicate::CONTAINS:
		return bg::relate(rhs, lhs, bg::de9im::mask("T*F**F***"));
	case BoostPredicate::COVERED_BY:
		return bg::relate(lhs, rhs, CoveredByMask());
	case BoostPredicate::COVERS:
		return bg::relate(rhs, lhs, CoveredByMask());
	case BoostPredicate::CONTAINS_PROPERLY:
		return bg::relate(lhs, rhs, bg::de9im::mask("T**FF*FF*"));
	case BoostPredicate::WITHIN_PROPERLY:
		return bg::relate(rhs, lhs, bg::de9im::mask("T**FF*FF*"));
	default:
		throw std::runtime_error("unknown spatial predicate");
	}
}

} // namespace

bool EvalPredicate(BoostPredicate pred, const BoostSingle &lhs, const BoostSingle &rhs) {
	return EvalSingle(pred, lhs, rhs);
}

bool EvalPredicate(BoostPredicate pred, const BoostGeometry &lhs, const BoostGeometry &rhs) {
	if (!lhs.IsCollection() && !rhs.IsCollection()) {
		return EvalSingle(pred, lhs.Single(), rhs.Single());
	}

	// Only INTERSECTS and DISJOINT decompose part-wise without recomputing the full DE-9IM matrix of the union.
	switch (pred) {
	case BoostPredicate::INTERSECTS:
		for (const auto &a : lhs.Parts()) {
			for (const auto &b : rhs.Parts()) {
				if (EvalSingle(BoostPredicate::INTERSECTS, a, b)) {
					return true;
				}
			}
		}
		return false;
	case BoostPredicate::DISJOINT:
		for (const auto &a : lhs.Parts()) {
			for (const auto &b : rhs.Parts()) {
				if (!EvalSingle(BoostPredicate::DISJOINT, a, b)) {
					return false;
				}
			}
		}
		return true;
	default:
		throw std::runtime_error("this predicate does not support GEOMETRYCOLLECTION arguments");
	}
}

} // namespace duckdb
