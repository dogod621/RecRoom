#pragma once

#include <iomanip>

#include "E57File.h"

namespace RecRoom
{
	template<class T>
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const T& pNode);

#define TYPE_SPACE std::left << std::setw(6) 
#define NAME_SPACE std::left << std::setw(12) 
#define NUMBER_SPACE std::left << std::setprecision(6) << std::setw(9) 

	// Parse Node
	template<> void OStreamE57NodeFormat<e57::StructureNode>(std::ostream& os, std::size_t pDepth, const e57::StructureNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "STRUCT" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.childCount() << std::endl;
		os << std::string(pDepth, '\t') << "{" << std::endl;
		pDepth++;

		//
		for (int64_t i = 0; i < pNode.childCount(); ++i)
			OStreamE57NodeFormat(os, pDepth, pNode.get(i));

		//
		os << std::string(pDepth - 1, '\t') << "}" << std::endl;
	}

	template<> void OStreamE57NodeFormat<e57::VectorNode>(std::ostream& os, std::size_t pDepth, const e57::VectorNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "VEC" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.childCount() << std::endl;
		os << std::string(pDepth, '\t') << "{" << std::endl;
		pDepth++;

		//
		for (int64_t i = 0; i < pNode.childCount(); ++i)
			OStreamE57NodeFormat(os, pDepth, pNode.get(i));

		//
		os << std::string(pDepth - 1, '\t') << "}" << std::endl;
	}

	template<> void OStreamE57NodeFormat<e57::CompressedVectorNode>(std::ostream& os, std::size_t pDepth, const e57::CompressedVectorNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "CP_VEV" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.childCount() << std::endl;
		os << std::string(pDepth, '\t') << "{" << std::endl;
		pDepth++;

		//
		OStreamE57NodeFormat(os, pDepth, pNode.prototype());
		OStreamE57NodeFormat(os, pDepth, pNode.codecs());

		//
		os << std::string(pDepth - 1, '\t') << "}" << std::endl;
	}

	template<> void OStreamE57NodeFormat<e57::IntegerNode>(std::ostream& os, std::size_t pDepth, const e57::IntegerNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "INT" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.value() << std::endl;
	}

	template<> void OStreamE57NodeFormat<e57::ScaledIntegerNode>(std::ostream& os, std::size_t pDepth, const e57::ScaledIntegerNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "SC_INT" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.scaledValue() << ", " << pNode.rawValue() << ", " << pNode.scale() << ", " << pNode.offset() << std::endl;
	}

	template<> void OStreamE57NodeFormat<const e57::FloatNode>(std::ostream& os, std::size_t pDepth, const e57::FloatNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "FLOAT" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.value() << std::endl;
	}

	template<> void OStreamE57NodeFormat<e57::StringNode>(std::ostream& os, std::size_t pDepth, const e57::StringNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "STR" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.value() << std::endl;
	}

	template<> void OStreamE57NodeFormat<e57::BlobNode>(std::ostream& os, std::size_t pDepth, const e57::BlobNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "BLOB" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.byteCount() << std::endl;
		os << std::string(pDepth, '\t') << "{" << std::endl;
		pDepth++;

		//
		os << std::string(pDepth - 1, '\t') << "}" << std::endl;
	}

	template<> void OStreamE57NodeFormat<e57::Node>(std::ostream& os, std::size_t pDepth, const e57::Node& pNode)
	{
		switch (pNode.type())
		{
		case e57::NodeType::E57_STRUCTURE: return OStreamE57NodeFormat(os, pDepth, e57::StructureNode(pNode));
		case e57::NodeType::E57_VECTOR: return OStreamE57NodeFormat(os, pDepth, e57::VectorNode(pNode));
		case e57::NodeType::E57_COMPRESSED_VECTOR: return OStreamE57NodeFormat(os, pDepth, e57::CompressedVectorNode(pNode));
		case e57::NodeType::E57_INTEGER: return OStreamE57NodeFormat(os, pDepth, e57::IntegerNode(pNode));
		case e57::NodeType::E57_SCALED_INTEGER: return OStreamE57NodeFormat(os, pDepth, e57::ScaledIntegerNode(pNode));
		case e57::NodeType::E57_FLOAT: return OStreamE57NodeFormat(os, pDepth, e57::FloatNode(pNode));
		case e57::NodeType::E57_STRING: return OStreamE57NodeFormat(os, pDepth, e57::StringNode(pNode));
		case e57::NodeType::E57_BLOB: return OStreamE57NodeFormat(os, pDepth, e57::BlobNode(pNode));
		default: return;
		}
	}

	std::ostream& operator << (std::ostream& os, const e57::ImageFile& v)
	{
		RecRoom::OStreamE57NodeFormat(os, 0, v.root());
	}
}