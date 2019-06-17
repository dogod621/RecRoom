#include <cmath>

#include "E57File.h"

namespace RecRoom
{
	//
	/*nlohmann::json ScanInfo::DumpToJson()
	{
		nlohmann::json j;

		// Save scan info
		j["scanner"] = ScannerToStr(scanner);
		j["coodSys"] = CoodSysToStr(coodSys);
		j["raeMode"] = RAEModeToStr(raeMode);

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

		j["ID"] = ID;

		return j;
	}

	ScanInfo ScanInfo::LoadFromJson(const nlohmann::json& j)
	{
		ScanInfo scanInfo;

		scanInfo.scanner = StrToScanner(j["scanner"].get<std::string>());
		scanInfo.coodSys = StrToCoodSys(j["coodSys"].get<std::string>());
		scanInfo.raeMode = StrToRAEMode(j["raeMode"].get<std::string>());

		nlohmann::json::const_iterator it = j["transform"].begin();
		for (int r = 0; r < 4; ++r)
		{
			for (int c = 0; c < 4; ++c)
			{
				scanInfo.transform(r, c) = *it;
				++it;
			}
		}

		it = j["position"].begin();
		scanInfo.position.x() = *it;
		++it;
		scanInfo.position.y() = *it;
		++it;
		scanInfo.position.z() = *it;
		++it;

		it = j["orientation"].begin();
		scanInfo.orientation.x() = *it;
		++it;
		scanInfo.orientation.y() = *it;
		++it;
		scanInfo.orientation.z() = *it;
		++it;
		scanInfo.orientation.w() = *it;
		++it;

		scanInfo.position_float = Eigen::Vector3f(scanInfo.position.x(), scanInfo.position.y(), scanInfo.position.z());
		scanInfo.orientation_float = Eigen::Quaternionf(scanInfo.orientation.w(), scanInfo.orientation.x(), scanInfo.orientation.y(), scanInfo.orientation.z());

		scanInfo.hasPointXYZ = j["hasPointXYZ"];
		scanInfo.hasPointRGB = j["hasPointRGB"];
		scanInfo.hasPointI = j["hasPointI"];

		scanInfo.numPoints = j["numPoints"];
		scanInfo.numValidPoints = j["numValidPoints"];

		scanInfo.ID = j["ID"];

		return scanInfo;
	}

	void Scan::Load(const e57::ImageFile& imf, const e57::VectorNode& data3D, int64_t scanID)
	{
		ID = scanID;
		e57::StructureNode scan(data3D.get(scanID));
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
				PCL_WARN("[e57::%s::Load] Scan didnot define pose translation.\n", "Scan");

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
				PCL_WARN("[e57::%s::Load] Scan didnot define pose rotation.\n", "Scan");
		}
		else
			PCL_WARN("[e57::%s::Load] Scan didnot define pose.\n", "Scan");

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
					coodSys = CoodSys::XYZ;
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
					coodSys = CoodSys::RAE;
					raeMode = RAEMode::E_X_Y; // E57 use this
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
						PCL_WARN("[e57::%s::Load] Failed to read E57 points, ignore the scan.\n", "Scan");
						hasPointXYZ = false;
						hasPointRGB = false;
						hasPointI = false;
					}
					reader.close();
				}
			}
			else
				PCL_WARN("[e57::%s::Load] Not supported scan points type.\n", "Scan");
		}
		else
			PCL_WARN("[e57::%s::Load] Scan didnot define points.\n", "Scan");
	}

	void Scan::ExtractValidPointCloud(pcl::PointCloud<PointE57>& scanCloud, const uint8_t minRGB)
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
					switch (coodSys)
					{
					case CoodSys::XYZ:
						sp.x = _x[pi];
						sp.y = _y[pi];
						sp.z = _z[pi];
						break;

					case CoodSys::RAE:
					{
						Eigen::Vector3d xyz = RAEToXYZ(raeMode, Eigen::Vector3d(_x[pi], _y[pi], _z[pi]));

						sp.x = xyz.x();
						sp.y = xyz.y();
						sp.z = xyz.z();
					}
					break;

					default:
						throw std::exception("Coordinate system invalid!!?");
						break;
					}
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
				if (sp.Valid() && (sp.r >= minRGB || sp.g >= minRGB || sp.b >= minRGB))
					scanCloud.push_back(sp);
			}
			pcl::transformPointCloud(scanCloud, scanCloud, transform);
			numValidPoints = scanCloud.size();
		}
	}*/
}