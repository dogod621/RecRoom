#pragma once

#include <iomanip>

#include "E57Data.h"

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

	//
	template<Scanner scanner>
	nlohmann::json E57ScanMeta<scanner>::DumpToJson()
	{
		nlohmann::json j;

		// Save scan info
		j["scanner"] = Convert<std::string, Scanner>(scanner);

		for (int r = 0; r < 4; ++r)
			for (int c = 0; c < 4; ++c)
				j["transform"].push_back(transform(r, c));

		j["position"].push_back(position.x());
		j["position"].push_back(position.y());
		j["position"].push_back(position.z());

		j["orientation"].push_back(orientation.x());
		j["orientation"].push_back(orientation.y());
		j["orientation"].push_back(orientation.z());
		j["orientation"].push_back(orientation.w());

		j["hasPointXYZ"] = hasPointXYZ;
		j["hasPointRGB"] = hasPointRGB;
		j["hasPointI"] = hasPointI;

		j["numPoints"] = numPoints;
		j["numValidPoints"] = numValidPoints;

		j["serialNumber"] = serialNumber;

		return j;
	}

	template<Scanner scanner>
	void E57ScanMeta<scanner>::LoadFromJson(const nlohmann::json& j)
	{
		scanner = Convert<Scanner, std::string>(j["scanner"].get<std::string>());
			
		nlohmann::json::const_iterator it = j["transform"].begin();
		for (int r = 0; r < 4; ++r)
		{
			for (int c = 0; c < 4; ++c)
			{
				transform(r, c) = *it;
				++it;
			}
		}

		it = j["position"].begin();
		position.x() = *it;
		++it;
		position.y() = *it;
		++it;
		position.z() = *it;
		++it;

		it = j["orientation"].begin();
		orientation.x() = *it;
		++it;
		orientation.y() = *it;
		++it;
		orientation.z() = *it;
		++it;
		orientation.w() = *it;
		++it;

		hasPointXYZ = j["hasPointXYZ"];
		hasPointRGB = j["hasPointRGB"];
		hasPointI = j["hasPointI"];

		numPoints = j["numPoints"];
		numValidPoints = j["numValidPoints"];

		serialNumber = j["serialNumber"];
	}

	//
	template<Scanner scanner>
	void E57ScanData<scanner>::Load(const e57::ImageFile& imf, const e57::VectorNode& data3D, int64_t serialNumber_)
	{
		serialNumber = serialNumber_;
		e57::StructureNode scan(data3D.get(serialNumber));
		std::vector<e57::SourceDestBuffer> sdBuffers;

		// Parse pose
		if (scan.isDefined("pose"))
		{
			e57::StructureNode scanPose(scan.get("pose"));
			if (scanPose.isDefined("translation"))
			{
				e57::StructureNode scanPoseTranslation(scanPose.get("translation"));

				position = Eigen::Vector3d(
					e57::FloatNode(scanPoseTranslation.get("x")).value(),
					e57::FloatNode(scanPoseTranslation.get("y")).value(),
					e57::FloatNode(scanPoseTranslation.get("z")).value());

				position_float = Eigen::Vector3f(position.x(), position.y(), position.z());

				transform.block(0, 3, 3, 1) = position;
			}
			else
				WARNING("Scan didnot define pose translation, use default value");

			if (scanPose.isDefined("rotation"))
			{
				e57::StructureNode scanPoseRotation(scanPose.get("rotation"));

				orientation = Eigen::Quaterniond(
					e57::FloatNode(scanPoseRotation.get("w")).value(),
					e57::FloatNode(scanPoseRotation.get("x")).value(),
					e57::FloatNode(scanPoseRotation.get("y")).value(),
					e57::FloatNode(scanPoseRotation.get("z")).value());

				orientation_float = Eigen::Quaternionf(orientation.w(), orientation.x(), orientation.y(), orientation.z());

				transform.block(0, 0, 3, 3) = orientation.toRotationMatrix();
			}
			else
				WARNING("Scan didnot define pose rotation, use default value");
		}
		else
			WARNING(" Scan didnot define pose, use default value");

		// Parse scale & offset
		if (scan.isDefined("points"))
		{
			e57::Node scanPointsNode = scan.get("points");

			if (scanPointsNode.type() == e57::NodeType::E57_COMPRESSED_VECTOR)
			{
				e57::CompressedVectorNode scanPoints(scanPointsNode);
				e57::StructureNode proto(scanPoints.prototype());
				std::shared_ptr<e57::Node> protoXNode;
				std::shared_ptr<e57::Node> protoYNode;
				std::shared_ptr<e57::Node> protoZNode;
				std::shared_ptr<e57::Node> protoRNode;
				std::shared_ptr<e57::Node> protoGNode;
				std::shared_ptr<e57::Node> protoBNode;
				std::shared_ptr<e57::Node> protoINode;
				numPoints = scanPoints.childCount();

				if (proto.isDefined("cartesianX") && proto.isDefined("cartesianY") && proto.isDefined("cartesianZ"))
				{
					coodSys = CoodSys::XYZ_PX_PY_PZ;// E57 use this
					hasPointXYZ = true;
					protoXNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("cartesianX")));
					protoYNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("cartesianY")));
					protoZNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("cartesianZ")));
					x = std::shared_ptr<float>(new float[numPoints]);
					y = std::shared_ptr<float>(new float[numPoints]);
					z = std::shared_ptr<float>(new float[numPoints]);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianX", x.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianY", y.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianZ", z.get(), numPoints, true, true));
				}
				else if (proto.isDefined("sphericalRange") && proto.isDefined("sphericalAzimuth") && proto.isDefined("sphericalElevation"))
				{
					coodSys = CoodSys::RAE_E_PX_PY;// E57 use this
					hasPointXYZ = true;
					protoXNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalRange")));
					protoYNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalAzimuth")));
					protoZNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalElevation")));
					x = std::shared_ptr<float>(new float[numPoints]);
					y = std::shared_ptr<float>(new float[numPoints]);
					z = std::shared_ptr<float>(new float[numPoints]);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalRange", x.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalAzimuth", y.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalElevation", z.get(), numPoints, true, true));
				}

				if (proto.isDefined("colorRed") && proto.isDefined("colorGreen") && proto.isDefined("colorBlue") && E57_CAN_CONTAIN_RGB)
				{
					hasPointRGB = true;
					protoRNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorRed")));
					protoGNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorGreen")));
					protoBNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorBlue")));
					r = std::shared_ptr<uint8_t>(new uint8_t[numPoints]);
					g = std::shared_ptr<uint8_t>(new uint8_t[numPoints]);
					b = std::shared_ptr<uint8_t>(new uint8_t[numPoints]);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorRed", r.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorGreen", g.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorBlue", b.get(), numPoints, true, true));
				}

				if (proto.isDefined("intensity") && E57_CAN_CONTAIN_INTENSITY)
				{
					hasPointI = true;
					protoINode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("intensity")));
					i = std::shared_ptr<float>(new float[numPoints]);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "intensity", i.get(), numPoints, true, true));
				}

				//
				if ((hasPointXYZ || hasPointRGB || hasPointI))
				{
					e57::CompressedVectorReader reader = scanPoints.reader(sdBuffers);
					if (reader.read() <= 0)
					{
						WARNING("Failed to read E57 points, ignore the scan");
						hasPointXYZ = false;
						hasPointRGB = false;
						hasPointI = false;
					}
					reader.close();
				}
			}
			else
				WARNING("Not supported scan points type, igore the scan");
		}
		else
			WARNING("Scan didnot define points, igore the scan");
	}

	template<Scanner scanner>
	void E57ScanData<scanner>::ToPointCloud(pcl::PointCloud<PointE57>& scanCloud)
	{
		if ((hasPointXYZ || hasPointRGB || hasPointI))
		{
			scanCloud.reserve(numPoints);
			float* _x = x.get();
			float* _y = y.get();
			float* _z = z.get();
			float* _i = i.get();
			uint8_t * _r = r.get();
			uint8_t * _g = g.get();
			uint8_t * _b = b.get();

			for (int64_t pi = 0; pi < numPoints; ++pi)
			{
				PointE57 sp;
				if (hasPointXYZ)
				{
					Eigen::Vector3d xyz = CoodConvert<CoodSys::XYZ_PX_PY_PZ, coodSys>(Eigen::Vector3d(_x[pi], _y[pi], _z[pi]));
					sp.x = xyz.x();
					sp.y = xyz.y();
					sp.z = xyz.z();
				}

#ifdef POINT_E57_WITH_RGB
				if (hasPointRGB)
				{
					sp.r = _r[pi];
					sp.g = _g[pi];
					sp.b = _b[pi];
				}
				else
				{
					sp.r = 255;
					sp.g = 255;
					sp.b = 255;
				}
#endif

#ifdef POINT_E57_WITH_INTENSITY
				if (hasPointI)
				{
					sp.intensity = _i[pi];
				}
#endif

#ifdef POINT_E57_WITH_LABEL
				sp.label = (uint32_t)ID;
#endif
				if (pcl::isFinite(sp))
					scanCloud.push_back(sp);
			}
			pcl::transformPointCloud(scanCloud, scanCloud, transform);
			numValidPoints = scanCloud.size();
		}
	}
}