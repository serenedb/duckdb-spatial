#include "spatial/modules/boost/boost_serde.hpp"

#include "spatial/util/binary_reader.hpp"
#include "spatial/util/binary_writer.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace duckdb {

namespace {

constexpr uint32_t TYPE_POINT = 1;
constexpr uint32_t TYPE_LINESTRING = 2;
constexpr uint32_t TYPE_POLYGON = 3;
constexpr uint32_t TYPE_MULTI_POINT = 4;
constexpr uint32_t TYPE_MULTI_LINESTRING = 5;
constexpr uint32_t TYPE_MULTI_POLYGON = 6;
constexpr uint32_t TYPE_GEOMETRY_COLLECTION = 7;

struct Header {
	uint32_t type;
	uint32_t vertex_size;
};

Header ReadHeader(BinaryReader &reader) {
	if (reader.Read<uint8_t>() != 1) {
		throw std::runtime_error("Only little-endian WKB is supported");
	}
	const auto meta = reader.Read<uint32_t>();
	const auto type = (meta & 0x0000FFFF) % 1000;
	const auto flag = (meta & 0x0000FFFF) / 1000;
	const bool has_z = (flag & 0x01) != 0;
	const bool has_m = (flag & 0x02) != 0;
	return Header {type, static_cast<uint32_t>(2 + has_z + has_m)};
}

BoostPoint ReadVertex(BinaryReader &reader, const uint32_t vertex_size) {
	const auto x = reader.Read<double>();
	const auto y = reader.Read<double>();
	for (uint32_t i = 2; i < vertex_size; i++) {
		reader.Read<double>();
	}
	return BoostPoint {x, y};
}

template <class RING>
void ReadRing(BinaryReader &reader, const uint32_t vertex_size, RING &out) {
	const auto count = reader.Read<uint32_t>();
	out.reserve(count);
	for (uint32_t i = 0; i < count; i++) {
		out.push_back(ReadVertex(reader, vertex_size));
	}
}

BoostPolygon ReadPolygon(BinaryReader &reader, const uint32_t vertex_size) {
	BoostPolygon poly;
	const auto ring_count = reader.Read<uint32_t>();
	if (ring_count == 0) {
		return poly;
	}
	ReadRing(reader, vertex_size, poly.outer());
	poly.inners().resize(ring_count - 1);
	for (uint32_t i = 1; i < ring_count; i++) {
		ReadRing(reader, vertex_size, poly.inners()[i - 1]);
	}
	return poly;
}

void ReadSingle(BinaryReader &reader, std::vector<BoostSingle> &out);

// A nested GEOMETRYCOLLECTION is flattened into the outer one: Boost.Geometry has no recursive collection type, and
// for every operation we support the nesting carries no meaning.
void ReadCollectionParts(BinaryReader &reader, std::vector<BoostSingle> &out) {
	const auto part_count = reader.Read<uint32_t>();
	for (uint32_t i = 0; i < part_count; i++) {
		ReadSingle(reader, out);
	}
}

void ReadSingle(BinaryReader &reader, std::vector<BoostSingle> &out) {
	const auto header = ReadHeader(reader);

	switch (header.type) {
	case TYPE_POINT: {
		const auto vertex = ReadVertex(reader, header.vertex_size);
		if (std::isnan(vertex.x()) && std::isnan(vertex.y())) {
			out.emplace_back(BoostMultiPoint {});
		} else {
			out.emplace_back(vertex);
		}
	} break;
	case TYPE_LINESTRING: {
		BoostLinestring line;
		ReadRing(reader, header.vertex_size, line);
		out.emplace_back(std::move(line));
	} break;
	case TYPE_POLYGON: {
		// WKB does not pin ring orientation, but the Boost model does, so every ring is normalised on the way in.
		auto poly = ReadPolygon(reader, header.vertex_size);
		bg::correct(poly);
		out.emplace_back(std::move(poly));
	} break;
	case TYPE_MULTI_POINT: {
		BoostMultiPoint multi;
		const auto part_count = reader.Read<uint32_t>();
		multi.reserve(part_count);
		for (uint32_t i = 0; i < part_count; i++) {
			const auto part = ReadHeader(reader);
			const auto vertex = ReadVertex(reader, part.vertex_size);
			if (!std::isnan(vertex.x()) || !std::isnan(vertex.y())) {
				multi.push_back(vertex);
			}
		}
		out.emplace_back(std::move(multi));
	} break;
	case TYPE_MULTI_LINESTRING: {
		BoostMultiLinestring multi;
		const auto part_count = reader.Read<uint32_t>();
		multi.resize(part_count);
		for (uint32_t i = 0; i < part_count; i++) {
			const auto part = ReadHeader(reader);
			ReadRing(reader, part.vertex_size, multi[i]);
		}
		out.emplace_back(std::move(multi));
	} break;
	case TYPE_MULTI_POLYGON: {
		BoostMultiPolygon multi;
		const auto part_count = reader.Read<uint32_t>();
		multi.reserve(part_count);
		for (uint32_t i = 0; i < part_count; i++) {
			const auto part = ReadHeader(reader);
			multi.push_back(ReadPolygon(reader, part.vertex_size));
		}
		bg::correct(multi);
		out.emplace_back(std::move(multi));
	} break;
	case TYPE_GEOMETRY_COLLECTION: {
		ReadCollectionParts(reader, out);
	} break;
	default:
		throw std::runtime_error("Unsupported geometry type in GEOMETRY blob");
	}
}

//----------------------------------------------------------------------------------------------------------------------
// Serialize
//----------------------------------------------------------------------------------------------------------------------
constexpr size_t HEADER_SIZE = sizeof(uint8_t) + sizeof(uint32_t);
constexpr size_t VERTEX_SIZE = sizeof(double) * 2;

size_t RingSize(const size_t count) {
	return sizeof(uint32_t) + count * VERTEX_SIZE;
}

size_t PolygonSize(const BoostPolygon &poly) {
	if (poly.outer().empty()) {
		return HEADER_SIZE + sizeof(uint32_t);
	}
	size_t size = HEADER_SIZE + sizeof(uint32_t) + RingSize(poly.outer().size());
	for (const auto &inner : poly.inners()) {
		size += RingSize(inner.size());
	}
	return size;
}

struct SizeVisitor {
	size_t operator()(const BoostPoint &) const {
		return HEADER_SIZE + VERTEX_SIZE;
	}
	size_t operator()(const BoostLinestring &line) const {
		return HEADER_SIZE + RingSize(line.size());
	}
	size_t operator()(const BoostPolygon &poly) const {
		return PolygonSize(poly);
	}
	size_t operator()(const BoostMultiPoint &multi) const {
		return HEADER_SIZE + sizeof(uint32_t) + multi.size() * (HEADER_SIZE + VERTEX_SIZE);
	}
	size_t operator()(const BoostMultiLinestring &multi) const {
		size_t size = HEADER_SIZE + sizeof(uint32_t);
		for (const auto &line : multi) {
			size += HEADER_SIZE + RingSize(line.size());
		}
		return size;
	}
	size_t operator()(const BoostMultiPolygon &multi) const {
		size_t size = HEADER_SIZE + sizeof(uint32_t);
		for (const auto &poly : multi) {
			size += PolygonSize(poly);
		}
		return size;
	}
};

void WriteHeader(BinaryWriter &writer, const uint32_t type) {
	writer.Write<uint8_t>(1);
	writer.Write<uint32_t>(type);
}

void WriteVertex(BinaryWriter &writer, const BoostPoint &vertex) {
	writer.Write<double>(vertex.x());
	writer.Write<double>(vertex.y());
}

template <class RING>
void WriteRing(BinaryWriter &writer, const RING &ring) {
	writer.Write<uint32_t>(static_cast<uint32_t>(ring.size()));
	for (const auto &vertex : ring) {
		WriteVertex(writer, vertex);
	}
}

void WritePolygon(BinaryWriter &writer, const BoostPolygon &poly) {
	WriteHeader(writer, TYPE_POLYGON);
	if (poly.outer().empty()) {
		writer.Write<uint32_t>(0);
		return;
	}
	writer.Write<uint32_t>(static_cast<uint32_t>(poly.inners().size() + 1));
	WriteRing(writer, poly.outer());
	for (const auto &inner : poly.inners()) {
		WriteRing(writer, inner);
	}
}

struct WriteVisitor {
	BinaryWriter &writer;

	void operator()(const BoostPoint &point) const {
		WriteHeader(writer, TYPE_POINT);
		WriteVertex(writer, point);
	}
	void operator()(const BoostLinestring &line) const {
		WriteHeader(writer, TYPE_LINESTRING);
		WriteRing(writer, line);
	}
	void operator()(const BoostPolygon &poly) const {
		WritePolygon(writer, poly);
	}
	void operator()(const BoostMultiPoint &multi) const {
		WriteHeader(writer, TYPE_MULTI_POINT);
		writer.Write<uint32_t>(static_cast<uint32_t>(multi.size()));
		for (const auto &point : multi) {
			WriteHeader(writer, TYPE_POINT);
			WriteVertex(writer, point);
		}
	}
	void operator()(const BoostMultiLinestring &multi) const {
		WriteHeader(writer, TYPE_MULTI_LINESTRING);
		writer.Write<uint32_t>(static_cast<uint32_t>(multi.size()));
		for (const auto &line : multi) {
			WriteHeader(writer, TYPE_LINESTRING);
			WriteRing(writer, line);
		}
	}
	void operator()(const BoostMultiPolygon &multi) const {
		WriteHeader(writer, TYPE_MULTI_POLYGON);
		writer.Write<uint32_t>(static_cast<uint32_t>(multi.size()));
		for (const auto &poly : multi) {
			WritePolygon(writer, poly);
		}
	}
};

} // namespace

BoostGeometry BoostSerde::Deserialize(const char *buffer, const size_t buffer_size) {
	BinaryReader peek(buffer, buffer_size);
	const bool is_collection = ReadHeader(peek).type == TYPE_GEOMETRY_COLLECTION;

	BinaryReader reader(buffer, buffer_size);
	std::vector<BoostSingle> parts;
	ReadSingle(reader, parts);

	if (is_collection) {
		return BoostGeometry(std::move(parts));
	}
	return BoostGeometry(std::move(parts.front()));
}

size_t BoostSerde::GetRequiredSize(const BoostGeometry &geom) {
	if (!geom.IsCollection()) {
		return boost::variant2::visit(SizeVisitor {}, geom.Single());
	}
	size_t size = HEADER_SIZE + sizeof(uint32_t);
	for (const auto &part : geom.Parts()) {
		size += boost::variant2::visit(SizeVisitor {}, part);
	}
	return size;
}

void BoostSerde::Serialize(const BoostGeometry &geom, char *buffer, const size_t buffer_size) {
	BinaryWriter writer(buffer, buffer_size);

	if (!geom.IsCollection()) {
		boost::variant2::visit(WriteVisitor {writer}, geom.Single());
		return;
	}

	WriteHeader(writer, TYPE_GEOMETRY_COLLECTION);
	writer.Write<uint32_t>(static_cast<uint32_t>(geom.Parts().size()));
	for (const auto &part : geom.Parts()) {
		boost::variant2::visit(WriteVisitor {writer}, part);
	}
}

} // namespace duckdb
