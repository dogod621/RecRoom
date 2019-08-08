#include <iomanip>

#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

#include "Common/AsyncProcess.h"

#include "ScannerPcE57.h"

namespace RecRoom
{
	//
	void ScannerPcE57::LoadScanMetaSet()
	{
		if (data3DE57)
		{
			scanMetaSet->resize(data3DE57->childCount());
			for (std::size_t i = 0; i < scanMetaSet->size(); ++i)
			{
				ScanMeta& scanMeta = (*scanMetaSet)[i];

				e57::StructureNode scan(data3DE57->get(i));
				scanMeta.serialNumber = i;

				// Parse pose
				if (scan.isDefined("pose"))
				{
					e57::StructureNode scanPose(scan.get("pose"));
					if (scanPose.isDefined("translation"))
					{
						e57::StructureNode scanPoseTranslation(scanPose.get("translation"));

						scanMeta.position = Eigen::Vector3d(
							e57::FloatNode(scanPoseTranslation.get("x")).value(),
							e57::FloatNode(scanPoseTranslation.get("y")).value(),
							e57::FloatNode(scanPoseTranslation.get("z")).value());

						scanMeta.transform.block(0, 3, 3, 1) = scanMeta.position;
					}
					else
						PRINT_WARNING("Scan didnot define pose translation, use default value");

					if (scanPose.isDefined("rotation"))
					{
						e57::StructureNode scanPoseRotation(scanPose.get("rotation"));

						scanMeta.orientation = Eigen::Quaterniond(
							e57::FloatNode(scanPoseRotation.get("w")).value(),
							e57::FloatNode(scanPoseRotation.get("x")).value(),
							e57::FloatNode(scanPoseRotation.get("y")).value(),
							e57::FloatNode(scanPoseRotation.get("z")).value());

						scanMeta.transform.block(0, 0, 3, 3) = scanMeta.orientation.toRotationMatrix();
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
						scanMeta.numPoints = scanPoints.childCount();

						if (proto.isDefined("cartesianX") && proto.isDefined("cartesianY") && proto.isDefined("cartesianZ"))
						{
							scanMeta.rawDataCoordSys = CoordSys::XYZ_PX_PY_PZ;// E57 use this
							scanMeta.hasPointXYZ = true;
						}
						else if (proto.isDefined("sphericalRange") && proto.isDefined("sphericalAzimuth") && proto.isDefined("sphericalElevation"))
						{
							scanMeta.rawDataCoordSys = CoordSys::RAE_PE_PX_PY;// E57 use this
							scanMeta.hasPointXYZ = true;
						}

						// E57 cannot contain normal
						scanMeta.hasPointNormal = false;

						if (proto.isDefined("colorRed") && proto.isDefined("colorGreen") && proto.isDefined("colorBlue"))
						{
							scanMeta.hasPointRGB = true;
						}

						if (proto.isDefined("intensity"))
						{
							scanMeta.hasPointI = true;
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
		: ScannerPc(containerPcRAW), AsyncAble(1), imageFileE57(nullptr), data3DE57(nullptr), images2DE57(nullptr)
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
				data3DE57 = CONST_PTR(e57::VectorNode) (new e57::VectorNode(data3DNode));
				LoadScanMetaSet();
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

	void ScannerPcE57::LoadPcRAW(int serialNumber, PcRAW& pc, bool local) const
	{
		if (!imageFileE57)
			THROW_EXCEPTION("imageFileE57 is not set");
		if (!data3DE57)
			THROW_EXCEPTION("data3DE57 is not set");

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

		const ScanMeta& scanMeta = getScanMeta(serialNumber);
		
		pc.clear();
		pc.reserve(scanMeta.numPoints);
		{
			e57::StructureNode scan(data3DE57->get(scanMeta.serialNumber));
			if (scan.isDefined("points"))
			{
				e57::Node scanPointsNode = scan.get("points");
				if (scanPointsNode.type() == e57::NodeType::E57_COMPRESSED_VECTOR)
				{
					std::vector<e57::SourceDestBuffer> sdBuffers;

					if (scanMeta.hasPointXYZ)
					{
						xBuffer.resize(scanMeta.numPoints);
						yBuffer.resize(scanMeta.numPoints);
						zBuffer.resize(scanMeta.numPoints);
						if (scanMeta.rawDataCoordSys == CoordSys::XYZ_PX_PY_PZ)
						{
							sdBuffers.push_back(e57::SourceDestBuffer(*imageFileE57, "cartesianX", &xBuffer[0], scanMeta.numPoints, true, true));
							sdBuffers.push_back(e57::SourceDestBuffer(*imageFileE57, "cartesianY", &yBuffer[0], scanMeta.numPoints, true, true));
							sdBuffers.push_back(e57::SourceDestBuffer(*imageFileE57, "cartesianZ", &zBuffer[0], scanMeta.numPoints, true, true));
						}
						else if (scanMeta.rawDataCoordSys == CoordSys::RAE_PE_PX_PY)
						{
							sdBuffers.push_back(e57::SourceDestBuffer(*imageFileE57, "sphericalRange", &xBuffer[0], scanMeta.numPoints, true, true));
							sdBuffers.push_back(e57::SourceDestBuffer(*imageFileE57, "sphericalAzimuth", &yBuffer[0], scanMeta.numPoints, true, true));
							sdBuffers.push_back(e57::SourceDestBuffer(*imageFileE57, "sphericalElevation", &zBuffer[0], scanMeta.numPoints, true, true));
						}
						else
							THROW_EXCEPTION("CoordSys is not support");
					}

					if (scanMeta.hasPointRGB)
					{
						rBuffer.resize(scanMeta.numPoints);
						gBuffer.resize(scanMeta.numPoints);
						bBuffer.resize(scanMeta.numPoints);
						sdBuffers.push_back(e57::SourceDestBuffer(*imageFileE57, "colorRed", &rBuffer[0], scanMeta.numPoints, true, true));
						sdBuffers.push_back(e57::SourceDestBuffer(*imageFileE57, "colorGreen", &gBuffer[0], scanMeta.numPoints, true, true));
						sdBuffers.push_back(e57::SourceDestBuffer(*imageFileE57, "colorBlue", &bBuffer[0], scanMeta.numPoints, true, true));
					}
					if (scanMeta.hasPointI)
					{
						iBuffer.resize(scanMeta.numPoints);
						sdBuffers.push_back(e57::SourceDestBuffer(*imageFileE57, "intensity", &iBuffer[0], scanMeta.numPoints, true, true));
					}

					if ((scanMeta.hasPointXYZ || scanMeta.hasPointRGB || scanMeta.hasPointI))
					{
						e57::CompressedVectorNode scanPoints(scanPointsNode);
						e57::CompressedVectorReader reader = scanPoints.reader(sdBuffers);
						if (reader.read() <= 0)
							THROW_EXCEPTION("Read data failed");
						reader.close();
					}
				}
				else
					THROW_EXCEPTION("E57 file scan point type is not support");
			}
			else
			{
				if(scanMeta.numPoints > 0)
					THROW_EXCEPTION("E57 file scan did not contain point data, but numPoints is not 0");
				PRINT_WARNING("E57 file scan did not contain point data, ignore");
			}
		}

		{
			for (std::size_t px = 0; px < scanMeta.numPoints; ++px)
			{
				PointRAW sp;
				if (scanMeta.hasPointXYZ)
				{
					Eigen::Vector3d xyz;
					switch (scanMeta.rawDataCoordSys)
					{
					case CoordSys::XYZ_PX_PY_PZ: xyz = Eigen::Vector3d(xBuffer[px], yBuffer[px], zBuffer[px]); break;
					case CoordSys::RAE_PE_PX_PY: xyz = CoodConvert<CoordSys::XYZ_PX_PY_PZ, CoordSys::RAE_PE_PX_PY>(Eigen::Vector3d(xBuffer[px], yBuffer[px], zBuffer[px])); break;
					default: 
						THROW_EXCEPTION("CoordSys is not support"); 
						break;
					}
					sp.x = xyz.x();
					sp.y = xyz.y();
					sp.z = xyz.z();
				}

				if (scanMeta.hasPointRGB)
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

				if (scanMeta.hasPointI)
				{
					sp.intensity = iBuffer[px];
				}

				sp.serialNumber = (uint32_t)scanMeta.serialNumber;

				if (Valid(sp))
					pc.push_back(sp);
			}
		}
		if(!local)
			pcl::transformPointCloud(pc, pc, scanMeta.transform);
	}

	//
	class AsyncGlobal_ShipPcRAWData : public AsyncGlobal
	{
	public:
		AsyncGlobal_ShipPcRAWData(const ScannerPcE57* scannerPcE57 = nullptr)
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

	class AsyncQuery_ShipPcRAWData : public AsyncQuery<AsyncGlobal_ShipPcRAWData>
	{
	public:
		int serialNumber;

		AsyncQuery_ShipPcRAWData(ScanMeta scanMeta = ScanMeta())
			: serialNumber(-1) {}

		virtual int Check(const AsyncGlobal_ShipPcRAWData& global) const
		{
			if (serialNumber < 0) return 1;
			if (serialNumber >= global.ptrScannerPcE57()->getData3DE57()->childCount()) return 2;
			return 0;
		}

		virtual std::string Info(const AsyncGlobal_ShipPcRAWData& global) const
		{
			std::stringstream strQuery;
			strQuery << global.ptrScannerPcE57()->getScanMeta(serialNumber);
			return strQuery.str();
		}
	};

	int AStep_ShipPcRAWData(const AsyncGlobal_ShipPcRAWData& global, const AsyncQuery_ShipPcRAWData& query, PcRAW& data)
	{
		PRINT_INFO("Load from E57 - Start");

		global.ptrScannerPcE57()->LoadPcRAW(query.serialNumber, data, false);

		std::stringstream ss;
		ss << "Load from E57 - End - pcSize: " << data.size();
		PRINT_INFO(ss.str());

		return 0;
	}

	int BStep_ShipPcRAWData(const AsyncGlobal_ShipPcRAWData& global, const AsyncQuery_ShipPcRAWData& query, PcRAW& data)
	{
		if (global.ptrScannerPcE57()->getPreprocessor())
		{
			PRINT_INFO("Preprocessing - Start");

			PTR(PcRAW)temp(new PcRAW);
			pcl::copyPointCloud(data, *temp);
			data.clear();

			PTR(KDTreeRAW)tempAcc(new KDTreeRAW);
			tempAcc->setInputCloud(temp);
			global.ptrScannerPcE57()->getPreprocessor()->Process(tempAcc, temp, nullptr, data);

			std::stringstream ss;
			ss << "Preprocessing - End - orgPcSize: " << temp->size() << ", pcSize: " << data.size();
			PRINT_INFO(ss.str());
		}
		return 0;
	}

	int CStep_ShipPcRAWData(AsyncGlobal_ShipPcRAWData& global, const AsyncQuery_ShipPcRAWData& query, const PcRAW& data)
	{
		PRINT_INFO("Merge to container - Start");
		
		PTR(PcRAW)temp(new PcRAW);
		pcl::copyPointCloud(data, *temp);

		global.ptrScannerPcE57()->getContainerPcRAW()->Merge(temp);

		std::stringstream ss;
		ss << "Merge to container - End - size: " << global.ptrScannerPcE57()->getContainerPcRAW()->Size();
		PRINT_INFO(ss.str());

		return 0;
	}

	void ScannerPcE57::ShipPcRAW() const
	{
		if (imageFileE57)
		{
			if (data3DE57)
			{
				AsyncGlobal_ShipPcRAWData global(this);

				std::vector<AsyncQuery_ShipPcRAWData> queries(scanMetaSet->size());
				for (std::size_t i = 0; i < data3DE57->childCount(); ++i)
					queries[i].serialNumber = i;

				AsyncProcess<AsyncGlobal_ShipPcRAWData, AsyncQuery_ShipPcRAWData, PcRAW>(
					global, queries,
					AStep_ShipPcRAWData, BStep_ShipPcRAWData, CStep_ShipPcRAWData,
					asyncSize);
			}
			else
				PRINT_WARNING("data3DE57 is not set, ignore");
		}
		else
			PRINT_WARNING("data3DE57 is not set, ignore");
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
		if (v.getImageFileE57())
			RecRoom::OStreamE57NodeFormat(os, 0, v.getImageFileE57()->root());
		return os;
	}
}