#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/auto_io.h>

#include "nlohmann/json.hpp"

#include "Common/PCLUtils.h"
#include "Filter/FilterPcRemoveDuplicate.h"
#include "Interpolator/InterpolatorPcNearest.h"

#include "ReconstructorPc.h"

namespace RecRoom
{
	ReconstructorPc::ReconstructorPc(
		boost::filesystem::path filePath_,
		const CONST_PTR(ScannerPc)& scanner,
		const PTR(ContainerPcNDF)& containerPcNDF,
		float res)
		: DumpAble("ReconstructorPc", filePath_), status(ReconstructStatus::ReconstructStatus_UNKNOWN), scanner(scanner), containerPcNDF(containerPcNDF), res(res),
		pcMED(new PcMED), mesh(new Mesh),
		downSampler(nullptr),
		interpolator(new InterpolatorPcNearest<PointMED, PointMED>),
		outlierRemover(nullptr),
		normalEstimator(nullptr),
		albedoEstimator(nullptr),
		sharpnessEstimator(nullptr),
		segmenter(nullptr),
		mesher(nullptr)
	{
		if (!scanner)
			THROW_EXCEPTION("scanner is not set");
		if (!containerPcNDF)
			THROW_EXCEPTION("containerPcNDF is not set");

		if (CheckExist())
		{
			Load();
		}
		else
		{
			Dump();
		}

		//
		if (!pcMED)
			THROW_EXCEPTION("pcMED is not created?")
		if (!mesh)
			THROW_EXCEPTION("mesh is not created?")
		if(!interpolator)
			THROW_EXCEPTION("interpolator is not created?")
	}

	void ReconstructorPc::RecPointCloud()
	{
		//
		if (status & ReconstructStatus::POINT_CLOUD)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		{
			status = ReconstructStatus::ReconstructStatus_UNKNOWN;
			pcMED->clear();

			ImplementRecPointCloud();

			if (downSampler)
			{
				PTR(AccMED) accMED(new KDTreeMED);
				accMED->setInputCloud(pcMED);
				downSampler->ProcessInOut(accMED, pcMED, nullptr);
			}

			status = (ReconstructStatus)(status | ReconstructStatus::POINT_CLOUD);
			Dump();
		}
	}

	void ReconstructorPc::RecPcNormal()
	{
		if (status & ReconstructStatus::PC_NORMAL)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if (!WITH_PERPOINT_NORMAL)
		{
			PRINT_WARNING("!WITH_PERPOINT_NORMAL, ignore. You must compile with INPUT_PERPOINT_NORMAL or OUTPUT_PERPOINT_NORMAL to enable this feature.");
		}
		else if (!WITH_PERPOINT_SERIAL_NUMBER)
		{
			PRINT_WARNING("!WITH_PERPOINT_SERIAL_NUMBER, ignore. You must compile with INPUT_PERPOINT_SERIAL_NUMBER to enable this feature.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else if (normalEstimator)
		{
			ImplementRecPcNormal();
			status = (ReconstructStatus)(status | ReconstructStatus::PC_NORMAL);
			Dump();
		}
		else
		{
			PRINT_WARNING("normalEstimator is not set, ignore it");
		}
	}

	void ReconstructorPc::RecPcAlbedo()
	{
		if (status & ReconstructStatus::PC_ALBEDO)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if (!WITH_INPUT_PERPOINT_INTENSITY)
		{
			PRINT_WARNING("!WITH_INPUT_PERPOINT_INTENSITY, ignore. You must compile with INPUT_PERPOINT_INTENSITY to enable this feature.");
		}
		else if (!WITH_PERPOINT_NORMAL)
		{
			PRINT_WARNING("!WITH_PERPOINT_NORMAL, ignore. You must compile with INPUT_PERPOINT_NORMAL or OUTPUT_PERPOINT_NORMAL to enable this feature.");
		}
		else if (!WITH_PERPOINT_SERIAL_NUMBER)
		{
			PRINT_WARNING("!WITH_PERPOINT_SERIAL_NUMBER, ignore. You must compile with INPUT_PERPOINT_SERIAL_NUMBER to enable this feature.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_NORMAL) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_NORMAL is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else if (albedoEstimator)
		{
			ImplementRecPcAlbedo();
			status = (ReconstructStatus)(status | ReconstructStatus::PC_ALBEDO);
			Dump();
		}
		else
		{
			PRINT_WARNING("albedoEstimater is not set, ignore it");
		}
	}

	void ReconstructorPc::RecPcSharpness()
	{
		if (status & ReconstructStatus::PC_SHARPNESS)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if (!WITH_INPUT_PERPOINT_INTENSITY)
		{
			PRINT_WARNING("!WITH_INPUT_PERPOINT_INTENSITY, ignore. You must compile with INPUT_PERPOINT_INTENSITY to enable this feature.");
		}
		else if (!WITH_PERPOINT_SHARPNESS)
		{
			PRINT_WARNING("!WITH_PERPOINT_SHARPNESS, ignore. You must compile with OUTPUT_PERPOINT_INTENSITY to enable this feature.");
		}
		else if (!WITH_PERPOINT_NORMAL)
		{
			PRINT_WARNING("!WITH_PERPOINT_NORMAL, ignore. You must compile with INPUT_PERPOINT_NORMAL or OUTPUT_PERPOINT_NORMAL to enable this feature.");
		}
		else if (!WITH_PERPOINT_SERIAL_NUMBER)
		{
			PRINT_WARNING("!WITH_PERPOINT_SERIAL_NUMBER, ignore. You must compile with INPUT_PERPOINT_SERIAL_NUMBER to enable this feature.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_NORMAL) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_NORMAL is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_ALBEDO) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_ALBEDO is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else if (sharpnessEstimator)
		{
			ImplementRecPcSharpness();
			status = (ReconstructStatus)(status | ReconstructStatus::PC_SHARPNESS);
			Dump();
		}
		else
		{
			PRINT_WARNING("sharpnessEstimator is not set, ignore it");
		}
	}

	void ReconstructorPc::RecPcSegment()
	{
		if (status & ReconstructStatus::PC_SEGMENT)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if (!WITH_PERPOINT_LABEL)
		{
			PRINT_WARNING("!WITH_PERPOINT_LABEL, ignore. You must compile with OUTPUT_PERPOINT_LABEL to enable this feature.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else if (segmenter)
		{
			ImplementRecPcSegment();
			status = (ReconstructStatus)(status | ReconstructStatus::PC_SEGMENT);
			Dump();
		}
		else
		{
			PRINT_WARNING("segmenter is not set, ignore it");
		}
	}

	void ReconstructorPc::RecSegMaterial()
	{
		if (status & ReconstructStatus::SEG_MATERIAL)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if (!WITH_INPUT_PERPOINT_INTENSITY)
		{
			PRINT_WARNING("!WITH_INPUT_PERPOINT_INTENSITY, ignore. You must compile with INPUT_PERPOINT_INTENSITY to enable this feature.");
		}
		else if (!WITH_PERPOINT_NORMAL)
		{
			PRINT_WARNING("!WITH_PERPOINT_NORMAL, ignore. You must compile with INPUT_PERPOINT_NORMAL or OUTPUT_PERPOINT_NORMAL to enable this feature.");
		}
		else if (!WITH_PERPOINT_SERIAL_NUMBER)
		{
			PRINT_WARNING("!WITH_PERPOINT_SERIAL_NUMBER, ignore. You must compile with INPUT_PERPOINT_SERIAL_NUMBER to enable this feature.");
		}
		else if (!WITH_PERPOINT_LABEL)
		{
			PRINT_WARNING("!WITH_PERPOINT_LABEL, ignore. You must compile with OUTPUT_PERPOINT_LABEL to enable this feature.");
		}
		else if (containerPcNDF->Size() != 0)
		{
			PRINT_WARNING("containerPcLF is already used, ignore");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_SEGMENT) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_SEGMENT is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_NORMAL) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_NORMAL is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else
		{
			ImplementRecSegMaterial();
			status = (ReconstructStatus)(status | ReconstructStatus::SEG_MATERIAL);
			Dump();
		}
	}

	void ReconstructorPc::RecMesh()
	{
		if (status & ReconstructStatus::MESH)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if (!WITH_PERPOINT_NORMAL)
		{
			PRINT_WARNING("!WITH_PERPOINT_NORMAL, ignore. You must compile with INPUT_PERPOINT_NORMAL or OUTPUT_PERPOINT_NORMAL to enable this feature.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_NORMAL) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_NORMAL is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else if(mesher)
		{
			ImplementRecMesh();
			status = (ReconstructStatus)(status | ReconstructStatus::MESH);
			Dump();
		}
		else
		{
			PRINT_WARNING("mesher is not set, ignore it");
		}
	}

	//
	void ReconstructorPc::VisualSegNDFs()
	{
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("VisualSegmentNDFs")))
		{
			boost::filesystem::create_directory(filePath / boost::filesystem::path("VisualSegmentNDFs"));
			PRINT_INFO("Create directory: " + (filePath / boost::filesystem::path("VisualSegmentNDFs")).string());
		}

		std::size_t width = 256;
		std::size_t height = 256;
		for (std::size_t segID = 0; segID < containerPcNDF->Size(); ++segID)
		{
			{
				std::stringstream ss;
				ss << "VisualSegmentNDFs : " << segID;
				PRINT_INFO(ss.str().c_str());
			}

			PTR(PcNDF) pcNDF = containerPcNDF->GetData(segID);
			PcNDF pcVisNDF;
			pcVisNDF.width = width;
			pcVisNDF.height = height;
			pcVisNDF.is_dense = false;
			pcVisNDF.resize(width * height);
			for (PcNDF::iterator it = pcVisNDF.begin(); it != pcVisNDF.end(); ++it)
			{
				it->x = 0.0;// use as counter
				it->y = 0.0;
				it->z = 0.0;
				it->normal_x = NAN;
				it->normal_y = NAN;
				it->normal_z = NAN;
				it->intensity = 0.0;
			}

			for (PcNDF::iterator it = pcNDF->begin(); it != pcNDF->end(); ++it)
			{
				Eigen::Vector2d uv = ToUV(ToMapping(UVMode::HEMISPHERE, CoordSys::XYZ_PX_PY_PZ), Eigen::Vector3d(it->normal_x, it->normal_y, it->normal_z));
				std::size_t col = uv.x() * (width - 1);
				std::size_t row = (1.0 - uv.y()) * (height - 1);
				std::size_t index = row * width + col;
				PointNDF& pVisNDF = pcVisNDF[index];
				{
					pVisNDF.x += 1;
					pVisNDF.intensity += it->intensity;
				}
			}

			std::size_t index = 0;
			for (PcNDF::iterator it = pcVisNDF.begin(); it != pcVisNDF.end(); ++it)
			{
				if (it->x > 0)
				{
					it->intensity /= it->x;
					it->x = 0.0f;

					std::size_t col = index % width;
					std::size_t row = index / width;
					Eigen::Vector2d uv(
						(((double)col) + 0.5/ (double)(width - 1)) * 2.0 - 1.0,
						(1.0 - ((double)row) + 0.5 / (double)(height - 1)) * 2.0 - 1.0);

					it->normal_x = uv.x();
					it->normal_y = uv.y();
					it->normal_z = std::sqrt(1.0 - uv.x()*uv.x() - uv.y()*uv.y());

				}

				index++;
			}

			{
				std::stringstream fileName;
				fileName << segID << "_Dir.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromNormalField<PointNDF> pcie;
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVisNDF, image))
					THROW_EXCEPTION("Failed to extract an image from Normal field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualSegmentNDFs") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				std::stringstream fileName;
				fileName << segID << "_Intensity.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromIntensityField<PointNDF> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
				pcie.setScalingFactor(255.f);
				if (!pcie.extract(pcVisNDF, image))
					THROW_EXCEPTION("Failed to extract an image from Intensity field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualSegmentNDFs") / boost::filesystem::path(fileName.str())).string(), image);
			}

		}
	}

	void ReconstructorPc::VisualRecAtts()
	{
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("VisualRecAtts")))
		{
			boost::filesystem::create_directory(filePath / boost::filesystem::path("VisualRecAtts"));
			PRINT_INFO("Create directory: " + (filePath / boost::filesystem::path("VisualRecAtts")).string());
		}

		if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
			THROW_EXCEPTION("POINT_CLOUD is not reconstructed yet.");
		if (pcMED->empty())
			THROW_EXCEPTION("pcMED is empty.");

		PTR(AccMED) accMED(new KDTreeMED);
		accMED->setInputCloud(pcMED);

		//
		for (std::vector<ScanMeta>::const_iterator it = scanner->getScanMetaSet()->begin(); it != scanner->getScanMetaSet()->end(); ++it)
		{
			{
				std::stringstream ss;
				ss << "VisualRecAtts : " << it->serialNumber;
				PRINT_INFO(ss.str().c_str());
			}

			//
			PTR(PcMED) pcRaw(new PcMED);
			PTR(PcMED) pcRec(new PcMED);
			PcMED pcVisRaw;
			PcMED pcVisRec;
			std::vector<ColorHDR> pcVisRawRGB;
			std::vector<ColorHDR> pcVisRecRGB;
			std::size_t width = scanner->ScanImageWidth() / 4;
			std::size_t height = scanner->ScanImageHeight() / 4;
			{
				pcVisRaw.width = width;
				pcVisRaw.height = height;
				pcVisRaw.is_dense = false;
				pcVisRaw.resize(width*height);
				pcVisRawRGB.resize(width*height);
				for (PcMED::iterator jt = pcVisRaw.begin(); jt != pcVisRaw.end(); ++jt)
				{
					jt->x = 0.0; // use as counter
					jt->y = 0.0;
					jt->z = NAN; // use as depth buffer
				}
				pcVisRec = pcVisRaw;
				pcVisRecRGB.resize(width*height);

				//
				{
					PcRAW pcRaw_;
					scanner->LoadPcRAW(it->serialNumber, pcRaw_, false);
					pcRaw->resize(pcRaw_.size());
					for (std::size_t px = 0; px < pcRaw_.size(); ++px)
						(*pcRaw)[px] = pcRaw_[px];
					(*pcRec) = (*pcRaw);
				}

				// Upsampling
				interpolator->ProcessInOut(accMED, pcRec, nullptr);
			}

			//
			Eigen::Matrix4d wordToScan = it->transform.inverse();
			for (std::size_t px = 0; px < pcRaw->size(); ++px)
			{
				PointMED& pRaw = (*pcRaw)[px];
				PointMED& pRec = (*pcRec)[px];

				Eigen::Vector4d xyz = wordToScan * Eigen::Vector4d(pRaw.x, pRaw.y, pRaw.z, 1.0);
				Eigen::Vector3d uvd = scanner->ToScanImageUVDepth(Eigen::Vector3d(xyz.x(), xyz.y(), xyz.z()));
				std::size_t col = uvd.x() * (width - 1);
				std::size_t row = (1.0 - uvd.y()) * (height - 1);
				std::size_t index = row * width + col;

				PointMED& pVisRaw = pcVisRaw[index];
				PointMED& pVisRec = pcVisRec[index];
				ColorHDR& pVisRawRGB = pcVisRawRGB[index];
				ColorHDR& pVisRecRGB = pcVisRecRGB[index];
				{
					pVisRaw.x += 1;
					pVisRec.x += 1;

					bool cloest = false;
					if (!std::isfinite(pVisRaw.z))
						cloest = true;
					else if (pVisRaw.z < uvd.z())
						cloest = true;
					if (cloest)
					{
						pVisRaw.z = uvd.z();
						pVisRec.z = uvd.z();
#ifdef WITH_INPUT_PERPOINT_SERIAL_NUMBER
						pVisRaw.label = pRaw.serialNumber;
#endif
#ifdef WITH_OUTPUT_PERPOINT_LABEL
						pVisRec.label = pRec.label;
#endif
					}

#ifdef WITH_INPUT_PERPOINT_RGB
					pVisRawRGB.r += (float)pRaw.r;
					pVisRawRGB.g += (float)pRaw.g;
					pVisRawRGB.b += (float)pRaw.b;
#endif
#ifdef WITH_OUTPUT_PERPOINT_RGB
					pVisRecRGB.r += (float)pRec.r;
					pVisRecRGB.g += (float)pRec.g;
					pVisRecRGB.b += (float)pRec.b;
#endif

#ifdef WITH_INPUT_PERPOINT_INTENSITY
					pVisRaw.intensity += pRaw.intensity;
#endif
#ifdef WITH_OUTPUT_PERPOINT_INTENSITY
					pVisRec.intensity += pRec.intensity;
#endif

#ifdef WITH_INPUT_PERPOINT_NORMAL
					pVisRaw.normal_x += pRaw.normal_x;
					pVisRaw.normal_y += pRaw.normal_y;
					pVisRaw.normal_z += pRaw.normal_z;
					pVisRaw.curvature += pRaw.curvature;
#endif

#ifdef WITH_OUTPUT_PERPOINT_NORMAL
					pVisRec.normal_x += pRec.normal_x;
					pVisRec.normal_y += pRec.normal_y;
					pVisRec.normal_z += pRec.normal_z;
					pVisRec.curvature += pRec.curvature;
#endif

#ifdef WITH_OUTPUT_PERPOINT_SHARPNESS
					pVisRec.sharpness += pRec.sharpness;
#endif
				}
			}

			for (std::size_t px = 0; px < pcVisRaw.size(); ++px)
			{
				PointMED& pVisRaw = pcVisRaw[px];
				PointMED& pVisRec = pcVisRec[px];
				ColorHDR& pVisRawRGB = pcVisRawRGB[px];
				ColorHDR& pVisRecRGB = pcVisRecRGB[px];

				if (pVisRaw.x > 0)
				{
#ifdef WITH_INPUT_PERPOINT_RGB
					pVisRaw.r = std::max(std::min(pVisRawRGB.r / pVisRaw.x, 255.0f), 0.0f);
					pVisRaw.g = std::max(std::min(pVisRawRGB.g / pVisRaw.x, 255.0f), 0.0f);
					pVisRaw.b = std::max(std::min(pVisRawRGB.b / pVisRaw.x, 255.0f), 0.0f);
#endif

#ifdef WITH_OUTPUT_PERPOINT_RGB
					pVisRec.r = std::max(std::min(pVisRecRGB.r / pVisRec.x, 255.0f), 0.0f);
					pVisRec.g = std::max(std::min(pVisRecRGB.g / pVisRec.x, 255.0f), 0.0f);
					pVisRec.b = std::max(std::min(pVisRecRGB.b / pVisRec.x, 255.0f), 0.0f);
#endif

#ifdef WITH_INPUT_PERPOINT_INTENSITY
					pVisRaw.intensity /= pVisRaw.x;
#endif

#ifdef WITH_OUTPUT_PERPOINT_INTENSITY
					pVisRec.intensity /= pVisRec.x;
#endif

#ifdef WITH_INPUT_PERPOINT_NORMAL
					pVisRaw.normal_x /= pVisRaw.x;
					pVisRaw.normal_y /= pVisRaw.x;
					pVisRaw.normal_z /= pVisRaw.x;
					pVisRaw.curvature /= pVisRaw.x;
#endif

#ifdef WITH_OUTPUT_PERPOINT_NORMAL
					pVisRec.normal_x /= pVisRec.x;
					pVisRec.normal_y /= pVisRec.x;
					pVisRec.normal_z /= pVisRec.x;
					pVisRec.curvature /= pVisRec.x;
#endif

#ifdef WITH_OUTPUT_PERPOINT_SHARPNESS
					pVisRec.sharpness /= pVisRec.x;
#endif
				}
			}

			// Z
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_raw_Depth.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromZField<PointMED> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
				if (!pcie.extract(pcVisRaw, image))
					THROW_EXCEPTION("Failed to extract an image from Depth field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_Depth.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromZField<PointMED> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
				if (!pcie.extract(pcVisRec, image))
					THROW_EXCEPTION("Failed to extract an image from Depth field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

#ifdef WITH_INPUT_PERPOINT_RGB
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_raw_RGB.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromRGBField<PointMED> pcie;
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVisRaw, image))
					THROW_EXCEPTION("Failed to extract an image from RGB field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

#ifdef WITH_OUTPUT_PERPOINT_RGB
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_RGB.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromRGBField<PointMED> pcie;
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVisRec, image))
					THROW_EXCEPTION("Failed to extract an image from RGB field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

#ifdef WITH_INPUT_PERPOINT_INTENSITY
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_raw_Intensity.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromIntensityField<PointMED> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
				pcie.setScalingFactor(255.f);
				if (!pcie.extract(pcVisRaw, image))
					THROW_EXCEPTION("Failed to extract an image from Intensity field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

#ifdef WITH_OUTPUT_PERPOINT_INTENSITY
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_Intensity.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromIntensityField<PointMED> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
				pcie.setScalingFactor(255.f);
				if (!pcie.extract(pcVisRec, image))
					THROW_EXCEPTION("Failed to extract an image from Intensity field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

#ifdef WITH_INPUT_PERPOINT_NORMAL
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_raw_Normal.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromNormalField<PointMED> pcie;
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVisRaw, image))
					THROW_EXCEPTION("Failed to extract an image from Normal field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_raw_Curvature.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromCurvatureField<PointMED> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
				if (!pcie.extract(pcVisRaw, image))
					THROW_EXCEPTION("Failed to extract an image from Curvature field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

#ifdef WITH_OUTPUT_PERPOINT_NORMAL
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_Normal.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromNormalField<PointMED> pcie;
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVisRec, image))
					THROW_EXCEPTION("Failed to extract an image from Normal field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_Curvature.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromCurvatureField<PointMED> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
				if (!pcie.extract(pcVisRec, image))
					THROW_EXCEPTION("Failed to extract an image from Curvature field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

#ifdef WITH_INPUT_PERPOINT_SERIAL_NUMBER
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_raw_SerialNumber.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromLabelField<PointMED> pcie;
				pcie.setColorMode(pcl::io::PointCloudImageExtractorFromLabelField<PointMED>::COLORS_RGB_GLASBEY);
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVisRaw, image))
					THROW_EXCEPTION("Failed to extract an image from Label field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

#ifdef WITH_OUTPUT_PERPOINT_LABEL
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_Label.png";

				pcl::PCLImage image;
				//pcl::io::PointCloudImageExtractorFromLabelField<PointVisAtt> pcie;
				pcl::io::PointCloudImageExtractorFromLabelField<PointMED> pcie;
				pcie.setColorMode(pcl::io::PointCloudImageExtractorFromLabelField<PointMED>::COLORS_RGB_GLASBEY);
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVisRec, image))
					THROW_EXCEPTION("Failed to extract an image from Label field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

			//
#ifdef WITH_OUTPUT_PERPOINT_SHARPNESS
			{
				for (std::size_t px = 0; px < pcVisRaw.size(); ++px)
				{
					PointMED& pVisRec = pcVisRec[px];
					pVisRec.z = pVisRec.sharpness;
				}

				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_Sharpness.png";


				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromZField<PointMED> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
				pcie.setScalingFactor(1.0);
				if (!pcie.extract(pcVisRec, image))
					THROW_EXCEPTION("Failed to extract an image from Sharpness field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif
		}
	}

	void ReconstructorPc::Load()
	{
		DumpAble::Load();
		if (boost::filesystem::exists(filePath / boost::filesystem::path("pcMED.pcd")))
			pcl::io::loadPCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED);
		if (boost::filesystem::exists(filePath / boost::filesystem::path("mesh.ply")))
		{
			//pcl::io::load((filePath / boost::filesystem::path("mesh.ply")).string(), *mesh);
			pcl::io::loadPLYFile((filePath / boost::filesystem::path("mesh.ply")).string(), *mesh);
			pcl::io::loadPCDFile((filePath / boost::filesystem::path("meshVertices.pcd")).string(), mesh->cloud);
		}
	};

	void ReconstructorPc::Dump() const
	{
		DumpAble::Dump();
		if (pcMED->size() > 0)
			pcl::io::savePCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED, true);
		if (!mesh->cloud.data.empty())
		{
			SaveAsPLY((filePath / boost::filesystem::path("mesh.ply")).string(), *mesh, 5, false);
			pcl::io::savePCDFile((filePath / boost::filesystem::path("meshVertices.pcd")).string(), mesh->cloud,
				Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);
		}
	};

	void ReconstructorPc::Load(const nlohmann::json& j)
	{
		if (j.find("res") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"res\"");
		res = j["res"];

		if (j.find("status") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"status\"");
		status = Convert<ReconstructStatus, nlohmann::json>(j["status"]);
	}

	void ReconstructorPc::Dump(nlohmann::json& j) const
	{
		j["res"] = res;
		j["status"] = Convert<nlohmann::json, ReconstructStatus>(status);
	}

	bool ReconstructorPc::CheckExist() const
	{
		if (!DumpAble::CheckExist())
			return false;
		return true;
	}


	void ReconstructorPc::ImplementRecPcSegment()
	{
		PTR(AccMED) accMED(new KDTreeMED);
		accMED->setInputCloud(pcMED);
		segmenter->ProcessInOut(accMED, pcMED, nullptr);
	}

	void ReconstructorPc::ImplementRecMesh()
	{
		PTR(PcREC) pcREC(new PcREC);
		pcREC->resize(pcMED->size());
		for (std::size_t px = 0; px < pcMED->size(); ++px)
			(*pcREC)[px] = (*pcMED)[px];

		PTR(AccREC) accREC(new KDTreeREC);
		accREC->setInputCloud(pcREC);
		mesher->Process(accREC, pcREC, nullptr, *mesh);
	}
}

