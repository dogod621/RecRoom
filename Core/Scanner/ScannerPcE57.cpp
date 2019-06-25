#include <iomanip>

#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

#include "Common/AsyncProcess.h"

#include "ScannerPcE57.h"

namespace RecRoom
{
	//
	void ScannerPcE57::LoadScanMeta()
	{
		if (data3DE57)
		{
			scanMeta.resize(data3DE57->childCount());
			for (std::size_t i = 0; i < scanMeta.size(); ++i)
			{
				scanMeta[i].scanner = Scanner::BLK360;

				e57::StructureNode scan(data3DE57->get(i));
				scanMeta[i].serialNumber = i;

				// Parse pose
				if (scan.isDefined("pose"))
				{
					e57::StructureNode scanPose(scan.get("pose"));
					if (scanPose.isDefined("translation"))
					{
						e57::StructureNode scanPoseTranslation(scanPose.get("translation"));

						scanMeta[i].position = Eigen::Vector3d(
							e57::FloatNode(scanPoseTranslation.get("x")).value(),
							e57::FloatNode(scanPoseTranslation.get("y")).value(),
							e57::FloatNode(scanPoseTranslation.get("z")).value());

						scanMeta[i].transform.block(0, 3, 3, 1) = scanMeta[i].position;
					}
					else
						PRINT_WARNING("Scan didnot define pose translation, use default value");

					if (scanPose.isDefined("rotation"))
					{
						e57::StructureNode scanPoseRotation(scanPose.get("rotation"));

						scanMeta[i].orientation = Eigen::Quaterniond(
							e57::FloatNode(scanPoseRotation.get("w")).value(),
							e57::FloatNode(scanPoseRotation.get("x")).value(),
							e57::FloatNode(scanPoseRotation.get("y")).value(),
							e57::FloatNode(scanPoseRotation.get("z")).value());

						scanMeta[i].transform.block(0, 0, 3, 3) = scanMeta[i].orientation.toRotationMatrix();
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
						scanMeta[i].numPoints = scanPoints.childCount();

						if (proto.isDefined("cartesianX") && proto.isDefined("cartesianY") && proto.isDefined("cartesianZ"))
						{
							scanMeta[i].rawDataCoordSys = CoordSys::XYZ_PX_PY_PZ;// E57 use this
							scanMeta[i].hasPointXYZ = true;
						}
						else if (proto.isDefined("sphericalRange") && proto.isDefined("sphericalAzimuth") && proto.isDefined("sphericalElevation"))
						{
							scanMeta[i].rawDataCoordSys = CoordSys::RAE_PE_PX_PY;// E57 use this
							scanMeta[i].hasPointXYZ = true;
						}

						// E57 cannot contain normal
						scanMeta[i].hasPointNormal = false;

						if (proto.isDefined("colorRed") && proto.isDefined("colorGreen") && proto.isDefined("colorBlue") && RAW_CAN_CONTAIN_RGB)
						{
							scanMeta[i].hasPointRGB = true;
						}

						if (proto.isDefined("intensity") && RAW_CAN_CONTAIN_INTENSITY)
						{
							scanMeta[i].hasPointI = true;
						}
					}
					else
						PRINT_WARNING("Not supported scan points type, igore the scan");
				}
				else
					PRINT_WARNING("Scan didnot define points, igore the scan");

			}
		}
	}

	ScannerPcE57::ScannerPcE57(
		const boost::filesystem::path& filePath,
		const PTR(ContainerPcRAW)& containerPcRAW)
		: ScannerPc(containerPcRAW), imageFileE57(nullptr), data3DE57(nullptr), images2DE57(nullptr)
	{
		if (!IsFileE57(filePath, true))
			THROW_EXCEPTION("filePath is not valid.");

		imageFileE57 = PTR(e57::ImageFile)(new e57::ImageFile(filePath.string().c_str(), "r"));

		//
		if (imageFileE57->root().isDefined("data3D"))
		{
			e57::Node data3DNode = imageFileE57->root().get("data3D");

			if (data3DNode.type() == e57::NodeType::E57_VECTOR)
			{
				data3DE57 = CONST_PTR(e57::VectorNode) ( new e57::VectorNode (data3DNode) );
				LoadScanMeta();
			}
			else
				PRINT_WARNING("E57 file data3D is not vector, ignore it");
		}
		else
			PRINT_WARNING("E57 file did not define data3D, ignore it");

		//
		if (imageFileE57->root().isDefined("images2D"))
		{
			e57::Node images2DNode = imageFileE57->root().get("images2D");

			if (images2DNode.type() == e57::NodeType::E57_VECTOR)
			{
				images2DE57 = CONST_PTR(e57::VectorNode) (new e57::VectorNode(images2DNode));
			}
			else
				PRINT_WARNING("E57 file images2D is not vector, ignore it");
		}
		else
			PRINT_WARNING("E57 file did not define images2D, ignore it");
	}

	//
	class AsyncGlobal_ShipData : public AsyncGlobal
	{
	public:
		AsyncGlobal_ShipData(const ScannerPcE57* scannerPcE57 = nullptr)
			: scannerPcE57(scannerPcE57) {}

		virtual int Check() const
		{
			if (scannerPcE57 == nullptr) return 1;
			if (!scannerPcE57->getImageFileE57()) return 2;
			if (!scannerPcE57->getData3DE57()) return 3;
			return 0;
		}

	public:
		const ScannerPcE57* ptrScannerPcE57() const
		{
			return scannerPcE57;
		}

	protected:
		const ScannerPcE57* scannerPcE57;
	};

	class AsyncQuery_ShipData : public AsyncQuery<AsyncGlobal_ShipData>
	{
	public:
		ScanMeta scanMeta;

		AsyncQuery_ShipData(ScanMeta scanMeta = ScanMeta())
			: scanMeta(scanMeta) {}

		virtual int Check(const AsyncGlobal_ShipData& global) const
		{
			if (scanMeta.serialNumber < 0) return 1;
			if (scanMeta.serialNumber >= global.ptrScannerPcE57()->getData3DE57()->childCount()) return 2;
			return 0;
		}

		virtual std::string Info(const AsyncGlobal_ShipData& global) const
		{
			std::stringstream strQuery;
			strQuery << scanMeta.serialNumber 
				<< " - Size:" << scanMeta.numPoints
				<< ", XYZ:" << scanMeta.hasPointXYZ 
				<< ", N:" << scanMeta.hasPointNormal
				<< ", RGB:" << scanMeta.hasPointRGB
				<< ", I:" << scanMeta.hasPointI;
			return strQuery.str();
		}
	};

	int AStep_ShipData(const AsyncGlobal_ShipData& global, const AsyncQuery_ShipData& query, PcRAW& data)
	{
		{
			PRINT_INFO("Load from E57 - Start");

			std::vector<e57::SourceDestBuffer> sdBuffers;

			std::vector<float> xBuffer;
			std::vector<float> yBuffer;
			std::vector<float> zBuffer;
			std::vector<float> normalXBuffer;
			std::vector<float> normalYBuffer;
			std::vector<float> normalZBuffer;
			std::vector<float> iBuffer;
			std::vector<uint8_t> rBuffer;
			std::vector<uint8_t> gBuffer;
			std::vector<uint8_t> bBuffer;

			if (query.scanMeta.hasPointXYZ)
			{
				xBuffer.resize(query.scanMeta.numPoints);
				yBuffer.resize(query.scanMeta.numPoints);
				zBuffer.resize(query.scanMeta.numPoints);
				if (query.scanMeta.rawDataCoordSys == CoordSys::XYZ_PX_PY_PZ)
				{
					sdBuffers.push_back(e57::SourceDestBuffer(*global.ptrScannerPcE57()->getImageFileE57(), "cartesianX", &xBuffer[0], query.scanMeta.numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(*global.ptrScannerPcE57()->getImageFileE57(), "cartesianY", &yBuffer[0], query.scanMeta.numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(*global.ptrScannerPcE57()->getImageFileE57(), "cartesianZ", &zBuffer[0], query.scanMeta.numPoints, true, true));
				}
				else if (query.scanMeta.rawDataCoordSys == CoordSys::RAE_PE_PX_PY)
				{
					sdBuffers.push_back(e57::SourceDestBuffer(*global.ptrScannerPcE57()->getImageFileE57(), "sphericalRange", &xBuffer[0], query.scanMeta.numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(*global.ptrScannerPcE57()->getImageFileE57(), "sphericalAzimuth", &yBuffer[0], query.scanMeta.numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(*global.ptrScannerPcE57()->getImageFileE57(), "sphericalElevation", &zBuffer[0], query.scanMeta.numPoints, true, true));
				}
				else
					return 2;
			}
			if (query.scanMeta.hasPointNormal)
				return 3;
			if (query.scanMeta.hasPointRGB)
			{
				rBuffer.resize(query.scanMeta.numPoints);
				gBuffer.resize(query.scanMeta.numPoints);
				bBuffer.resize(query.scanMeta.numPoints);
				sdBuffers.push_back(e57::SourceDestBuffer(*global.ptrScannerPcE57()->getImageFileE57(), "colorRed", &rBuffer[0], query.scanMeta.numPoints, true, true));
				sdBuffers.push_back(e57::SourceDestBuffer(*global.ptrScannerPcE57()->getImageFileE57(), "colorGreen", &gBuffer[0], query.scanMeta.numPoints, true, true));
				sdBuffers.push_back(e57::SourceDestBuffer(*global.ptrScannerPcE57()->getImageFileE57(), "colorBlue", &bBuffer[0], query.scanMeta.numPoints, true, true));
			}
			if (query.scanMeta.hasPointI)
			{
				iBuffer.resize(query.scanMeta.numPoints);
				sdBuffers.push_back(e57::SourceDestBuffer(*global.ptrScannerPcE57()->getImageFileE57(), "intensity", &iBuffer[0], query.scanMeta.numPoints, true, true));
			}

			if ((query.scanMeta.hasPointXYZ || query.scanMeta.hasPointRGB || query.scanMeta.hasPointI))
			{
				e57::CompressedVectorNode scanPoints(*global.ptrScannerPcE57()->getData3DE57());
				e57::CompressedVectorReader reader = scanPoints.reader(sdBuffers);
				if (reader.read() <= 0)
				{
					return 4;
				}
				reader.close();
			}
			for (std::size_t px = 0; px < query.scanMeta.numPoints; ++px)
			{
				PointRAW sp;
				if (query.scanMeta.hasPointXYZ)
				{
					Eigen::Vector3d xyz;
					switch (query.scanMeta.rawDataCoordSys)
					{
					case CoordSys::XYZ_PX_PY_PZ: xyz = Eigen::Vector3d(xBuffer[px], yBuffer[px], zBuffer[px]); break;
					case CoordSys::RAE_PE_PX_PY: xyz = CoodConvert<CoordSys::XYZ_PX_PY_PZ, CoordSys::RAE_PE_PX_PY>(Eigen::Vector3d(xBuffer[px], yBuffer[px], zBuffer[px])); break;
					default: return 5;; break;
					}
					sp.x = xyz.x();
					sp.y = xyz.y();
					sp.z = xyz.z();
				}

#ifdef POINT_RAW_WITH_RGB
				if (query.scanMeta.hasPointRGB)
				{
					sp.r = rBuffer[px];
					sp.g = gBuffer[px];
					sp.b = bBuffer[px];
				}
				else
				{
					sp.r = 255;
					sp.g = 255;
					sp.b = 255;
				}
#endif

#ifdef POINT_RAW_WITH_INTENSITY
				if (query.scanMeta.hasPointI)
				{
					sp.intensity = iBuffer[px];
				}
#endif

#ifdef POINT_RAW_WITH_LABEL
				sp.label = (uint32_t)query.scanMeta.serialNumber;
#endif
				if (pcl::isFinite(sp))
					data.push_back(sp);
			}
			
			std::stringstream ss;
			ss << "Load from E57 - End - pcSize: " << data.size();
			PRINT_INFO(ss.str());
		}

		{
			std::stringstream ss;
			ss << "Transform to world - Start - transform: " << query.scanMeta.transform;
			PRINT_INFO(ss.str());

			pcl::transformPointCloud(data, data, query.scanMeta.transform);

			PRINT_INFO("Transform to world - End");
		}

		return 0;
	}

	int BStep_ShipData(const AsyncGlobal_ShipData& global, const AsyncQuery_ShipData& query, PcRAW& data)
	{
		if (global.ptrScannerPcE57()->getPreprocessor())
		{
			PTR(PcRAW)temp(new PcRAW);
			pcl::copyPointCloud(data, *temp);
			data.clear();
			global.ptrScannerPcE57()->getPreprocessor()->Process(nullptr, nullptr, temp, nullptr, data);
		}
		return 0;
	}

	int CStep_ShipData(AsyncGlobal_ShipData& global, const AsyncQuery_ShipData& query, const PcRAW& data)
	{
		PTR(PcRAW)temp(new PcRAW);
		pcl::copyPointCloud(data, *temp);
		global.ptrScannerPcE57()->getContainerPcRAW()->Merge(temp);
		return 0;
	}

	void ScannerPcE57::ShipData() const
	{
		if (imageFileE57)
		{
			if (data3DE57)
			{
				AsyncGlobal_ShipData global(this);

				std::vector<AsyncQuery_ShipData> queries(scanMeta.size());
				for (std::size_t i = 0; i < scanMeta.size(); ++i)
					queries[i].scanMeta = scanMeta[i];

				AsyncProcess<AsyncGlobal_ShipData, AsyncQuery_ShipData, PcRAW, 1>(
					global, queries,
					AStep_ShipData, BStep_ShipData, CStep_ShipData);
			}
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

	std::ostream& operator << (std::ostream& os, const ScannerPcE57& v)
	{
		if(v.getImageFileE57())
			RecRoom::OStreamE57NodeFormat(os, 0, v.getImageFileE57()->root());
		return os;
	}
}