#include <pcl/common/transforms.h>

#include "E57Data.h"

namespace RecRoom
{
	//
	void E57ScanData::ClearBuffers()
	{
		xBuffer.clear();
		yBuffer.clear();
		zBuffer.clear();
		iBuffer.clear();
		rBuffer.clear();
		gBuffer.clear();
		bBuffer.clear();
	}

	void E57ScanData::FromJson(const nlohmann::json& j)
	{
		ScanData<PointE57>::FromJson(j);
		hasPointXYZ = j["hasPointXYZ"];
		hasPointRGB = j["hasPointRGB"];
		hasPointI = j["hasPointI"];
		/*xBuffer.resize(j["xBuffer"].size());
		yBuffer.resize(j["yBuffer"].size());
		zBuffer.resize(j["zBuffer"].size());
		iBuffer.resize(j["iBuffer"].size());
		rBuffer.resize(j["rBuffer"].size());
		gBuffer.resize(j["gBuffer"].size());
		bBuffer.resize(j["bBuffer"].size());
		for (std::size_t px = 0; px < xBuffer.size(); ++px)
			xBuffer[px] = j["xBuffer"][px];
		for (std::size_t px = 0; px < yBuffer.size(); ++px)
			yBuffer[px] = j["yBuffer"][px];
		for (std::size_t px = 0; px < zBuffer.size(); ++px)
			zBuffer[px] = j["zBuffer"][px];
		for (std::size_t px = 0; px < iBuffer.size(); ++px)
			iBuffer[px] = j["iBuffer"][px];
		for (std::size_t px = 0; px < rBuffer.size(); ++px)
			rBuffer[px] = j["rBuffer"][px];
		for (std::size_t px = 0; px < gBuffer.size(); ++px)
			gBuffer[px] = j["gBuffer"][px];
		for (std::size_t px = 0; px < bBuffer.size(); ++px)
			bBuffer[px] = j["bBuffer"][px];*/
		xBuffer = j["xBuffer"].get<std::vector<float>>();
		yBuffer = j["yBuffer"].get<std::vector<float>>();
		zBuffer = j["zBuffer"].get<std::vector<float>>();
		iBuffer = j["iBuffer"].get<std::vector<float>>();
		rBuffer = j["rBuffer"].get<std::vector<uint8_t>>();
		gBuffer = j["gBuffer"].get<std::vector<uint8_t>>();
		bBuffer = j["bBuffer"].get<std::vector<uint8_t>>();
	}

	void E57ScanData::ToJson(nlohmann::json& j) const
	{
		ScanData<PointE57>::ToJson(j);
		j["hasPointXYZ"] = hasPointXYZ;
		j["hasPointRGB"] = hasPointRGB;
		j["hasPointI"] = hasPointI;
		j["xBuffer"] = xBuffer;
		j["yBuffer"] = yBuffer;
		j["zBuffer"] = zBuffer;
		j["iBuffer"] = iBuffer;
		j["rBuffer"] = rBuffer;
		j["gBuffer"] = gBuffer;
		j["bBuffer"] = bBuffer;
	}

	//
	void E57ScanData::FromE57Format(const e57::ImageFile& imf, const e57::VectorNode& data3D, int64_t serialNumber_)
	{
		//
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

				transform.block(0, 3, 3, 1) = position;
			}
			else
				PRINT_WARNING("Scan didnot define pose translation, use default value");

			if (scanPose.isDefined("rotation"))
			{
				e57::StructureNode scanPoseRotation(scanPose.get("rotation"));

				orientation = Eigen::Quaterniond(
					e57::FloatNode(scanPoseRotation.get("w")).value(),
					e57::FloatNode(scanPoseRotation.get("x")).value(),
					e57::FloatNode(scanPoseRotation.get("y")).value(),
					e57::FloatNode(scanPoseRotation.get("z")).value());

				transform.block(0, 0, 3, 3) = orientation.toRotationMatrix();
			}
			else
				PRINT_WARNING("Scan didnot define pose rotation, use default value");
		}
		else
			PRINT_WARNING(" Scan didnot define pose, use default value");

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
					xBuffer.resize(numPoints);
					yBuffer.resize(numPoints);
					zBuffer.resize(numPoints);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianX", &xBuffer[0], numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianY", &yBuffer[0], numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianZ", &zBuffer[0], numPoints, true, true));
				}
				else if (proto.isDefined("sphericalRange") && proto.isDefined("sphericalAzimuth") && proto.isDefined("sphericalElevation"))
				{
					coodSys = CoodSys::RAE_PE_PX_PY;// E57 use this
					hasPointXYZ = true;
					protoXNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalRange")));
					protoYNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalAzimuth")));
					protoZNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalElevation")));
					xBuffer.resize(numPoints);
					yBuffer.resize(numPoints);
					zBuffer.resize(numPoints);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalRange", &xBuffer[0], numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalAzimuth", &yBuffer[0], numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalElevation", &zBuffer[0], numPoints, true, true));
				}

				if (proto.isDefined("colorRed") && proto.isDefined("colorGreen") && proto.isDefined("colorBlue") && E57_CAN_CONTAIN_RGB)
				{
					hasPointRGB = true;
					protoRNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorRed")));
					protoGNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorGreen")));
					protoBNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorBlue")));
					rBuffer.resize(numPoints);
					gBuffer.resize(numPoints);
					bBuffer.resize(numPoints);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorRed", &rBuffer[0], numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorGreen", &gBuffer[0], numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorBlue", &bBuffer[0], numPoints, true, true));
				}

				if (proto.isDefined("intensity") && E57_CAN_CONTAIN_INTENSITY)
				{
					hasPointI = true;
					protoINode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("intensity")));
					iBuffer.resize(numPoints);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "intensity", &iBuffer[0], numPoints, true, true));
				}

				//
				if ((hasPointXYZ || hasPointRGB || hasPointI))
				{
					e57::CompressedVectorReader reader = scanPoints.reader(sdBuffers);
					if (reader.read() <= 0)
					{
						PRINT_WARNING("Failed to read E57 points, ignore the scan");
						hasPointXYZ = false;
						hasPointRGB = false;
						hasPointI = false;
					}
					reader.close();
				}
			}
			else
				PRINT_WARNING("Not supported scan points type, igore the scan");
		}
		else
			PRINT_WARNING("Scan didnot define points, igore the scan");
	}

	void E57ScanData::ToPointCloud(pcl::PointCloud<PointE57>& scanCloud) const
	{
		if ((hasPointXYZ || hasPointRGB || hasPointI))
		{
			scanCloud.reserve(numPoints);
			for (int64_t pi = 0; pi < numPoints; ++pi)
			{
				PointE57 sp;
				if (hasPointXYZ)
				{
					Eigen::Vector3d xyz;
					switch (coodSys)
					{
					case CoodSys::XYZ_PX_PY_PZ: xyz = Eigen::Vector3d(xBuffer[pi], yBuffer[pi], zBuffer[pi]); break;
					case CoodSys::RAE_PE_PX_PY: xyz = CoodConvert<CoodSys::XYZ_PX_PY_PZ, CoodSys::RAE_PE_PX_PY>(Eigen::Vector3d(xBuffer[pi], yBuffer[pi], zBuffer[pi])); break;
					default: THROW_EXCEPTION("coodSys is invalid"); break;
					}
					sp.x = xyz.x();
					sp.y = xyz.y();
					sp.z = xyz.z();
				}

#ifdef POINT_E57_WITH_RGB
				if (hasPointRGB)
				{
					sp.r = rBuffer[pi];
					sp.g = gBuffer[pi];
					sp.b = bBuffer[pi];
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
					sp.intensity = iBuffer[pi];
				}
#endif

#ifdef POINT_E57_WITH_LABEL
				sp.label = (uint32_t)serialNumber;
#endif
				if (pcl::isFinite(sp))
					scanCloud.push_back(sp);
			}
			pcl::transformPointCloud(scanCloud, scanCloud, transform);
		}
	}

	//
#define TYPE_SPACE std::left << std::setw(6) 
#define NAME_SPACE std::left << std::setw(12) 
#define NUMBER_SPACE std::left << std::setprecision(6) << std::setw(9) 

	// Parse Node
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::StructureNode& pNode)
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

	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::VectorNode& pNode)
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

	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::CompressedVectorNode& pNode)
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

	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::IntegerNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "INT" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.value() << std::endl;
	}

	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::ScaledIntegerNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "SC_INT" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.scaledValue() << ", " << pNode.rawValue() << ", " << pNode.scale() << ", " << pNode.offset() << std::endl;
	}

	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::FloatNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "FLOAT" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.value() << std::endl;
	}

	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::StringNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "STR" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.value() << std::endl;
	}

	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::BlobNode& pNode)
	{
		os << std::string(pDepth, '\t') << TYPE_SPACE << "BLOB" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.byteCount() << std::endl;
		os << std::string(pDepth, '\t') << "{" << std::endl;
		pDepth++;

		//
		os << std::string(pDepth - 1, '\t') << "}" << std::endl;
	}

	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::Node& pNode)
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
		return os;
	}
}