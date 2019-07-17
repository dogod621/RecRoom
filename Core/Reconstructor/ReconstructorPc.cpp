#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/ply_io.h>

#include "nlohmann/json.hpp"

#include "ReconstructorPc.h"

namespace RecRoom
{
	ReconstructorPc::ReconstructorPc(
		boost::filesystem::path filePath_,
		const CONST_PTR(ScannerPc)& scanner,
		const PTR(ContainerPcNDF)& containerPcNDF)
		: DumpAble("ReconstructorPc", filePath_), status(ReconstructStatus::ReconstructStatus_UNKNOWN), scanner(scanner), containerPcNDF(containerPcNDF), pcMED(new PcMED), mesh(new pcl::PolygonMesh)
	{
		if (!scanner)
			THROW_EXCEPTION("scanner is not set");
		if (!containerPcNDF)
			THROW_EXCEPTION("containerPcNDF is not set");

		if (this->CheckExist())
		{
			this->Load();
		}
		else
		{
			this->Dump();
		}

		//
		if (!pcMED)
			THROW_EXCEPTION("pcMED is not created?")
		if (!mesh)
			THROW_EXCEPTION("mesh is not created?")
	}

	void ReconstructorPc::DoRecPointCloud()
	{
		//
		if (status & ReconstructStatus::POINT_CLOUD)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		{
			status = ReconstructStatus::ReconstructStatus_UNKNOWN;
			pcMED->clear();
			RecPointCloud();
			status = (ReconstructStatus)(status | ReconstructStatus::POINT_CLOUD);
			this->Dump();
		}
	}

	void ReconstructorPc::DoRecPcAlbedo()
	{
		if (status & ReconstructStatus::PC_ALBEDO)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if (!RAW_CAN_CONTAIN_LABEL)
		{
			PRINT_WARNING("!RAW_CAN_CONTAIN_LABEL, ignore. You must compile with POINT_RAW_WITH_LABEL to enable this feature.");
		}
		else if (!RAW_CAN_CONTAIN_INTENSITY)
		{
			PRINT_WARNING("!RAW_CAN_CONTAIN_INTENSITY, ignore. You must compile with POINT_RAW_WITH_INTENSITY to enable this feature.");
		}
		else if (!MED_CAN_CONTAIN_NORMAL)
		{
			PRINT_WARNING("!MED_CAN_CONTAIN_NORMAL, ignore. You must compile with POINT_REC_WITH_NORMAL or POINT_RAW_WITH_NORMAL to enable this feature.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("pcMED is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else if(albedoEstimator)
		{
			RecPcAlbedo();
			status = (ReconstructStatus)(status | ReconstructStatus::PC_ALBEDO);
			this->Dump();
		}
		else
		{
			PRINT_WARNING("albedoEstimater is not set, ignore it");
		}
	}

	void ReconstructorPc::DoRecPcSegment()
	{
		if (status & ReconstructStatus::PC_SEGMENT)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if (!REC_CAN_CONTAIN_LABEL)
		{
			PRINT_WARNING("!REC_CAN_CONTAIN_LABEL, ignore. You must compile with POINT_REC_WITH_LABEL to enable this feature.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("pcMED is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else if (segmenter)
		{
			RecPcSegment();
			status = (ReconstructStatus)(status | ReconstructStatus::PC_SEGMENT);
			this->Dump();
		}
		else
		{
			PRINT_WARNING("segmenter is not set, ignore it");
		}
	}

	void ReconstructorPc::DoRecSegNDF()
	{
		if (status & ReconstructStatus::SEG_NDF)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if (!RAW_CAN_CONTAIN_LABEL)
		{
			PRINT_WARNING("!RAW_CAN_CONTAIN_LABEL, ignore. You must compile with POINT_RAW_WITH_LABEL to enable this feature.");
		}
		else if (!REC_CAN_CONTAIN_LABEL)
		{
			PRINT_WARNING("!REC_CAN_CONTAIN_LABEL, ignore. You must compile with POINT_REC_WITH_LABEL to enable this feature.");
		}
		else if (!MED_CAN_CONTAIN_NORMAL)
		{
			PRINT_WARNING("!MED_CAN_CONTAIN_NORMAL, ignore. You must compile with POINT_REC_WITH_NORMAL or POINT_RAW_WITH_NORMAL to enable this feature.");
		}
		else if (containerPcNDF->Size() != 0)
		{
			PRINT_WARNING("containerPcLF is already used, ignore");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("pcMED is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_SEGMENT) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("pcMED segment is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else
		{
			RecSegNDF();
			status = (ReconstructStatus)(status | ReconstructStatus::SEG_NDF);
			this->Dump();
		}
	}

	void ReconstructorPc::DoRecMesh()
	{
		if (status & ReconstructStatus::MESH)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if (!REC_CAN_CONTAIN_NORMAL)
		{
			PRINT_WARNING("!REC_CAN_CONTAIN_NORMAL, ignore. You must compile with POINT_REC_WITH_NORMAL to enable this feature.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("pcMED is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else if(mesher)
		{
			RecMesh();
			status = (ReconstructStatus)(status | ReconstructStatus::MESH);
			this->Dump();
		}
		else
		{
			PRINT_WARNING("mesher is not set, ignore it");
		}
	}

	//
	void ReconstructorPc::VisualSegmentNDFs()
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


		if (!upSampler)
			THROW_EXCEPTION("upSampler is not set");
		if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
			THROW_EXCEPTION("pcMED is not reconstructed yet.");
		if (pcMED->empty())
			THROW_EXCEPTION("pcMED is empty.");

		PTR(KDTreeMED) accMED(new KDTreeMED);
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
				PcIndex upIdx;
				upSampler->Process(accMED, pcMED, pcRec, upIdx);

				for (std::size_t px = 0; px < upIdx.size(); ++px)
				{
					if (upIdx[px] >= 0)
					{
						PointMED& tarP = (*pcRec)[px];
						PointMED& srcP = (*pcMED)[upIdx[px]];

#ifdef POINT_REC_WITH_NORMAL
						tarP.normal_x = srcP.normal_x;
						tarP.normal_y = srcP.normal_y;
						tarP.normal_z = srcP.normal_z;
						tarP.curvature = srcP.curvature;
#endif

#ifdef POINT_REC_WITH_INTENSITY
						tarP.intensity = srcP.intensity;
#endif

#ifdef POINT_REC_WITH_LABEL
						tarP.segLabel = srcP.segLabel;
						tarP.label = srcP.segLabel;
#endif		
					}
				}
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

					//
					bool cloest = false;
					if (!std::isfinite(pVisRaw.z))
						cloest = true;
					else if (pVisRaw.z < uvd.z())
						cloest = true;
					if (cloest)
					{
						pVisRaw.z = uvd.z();
						pVisRec.z = uvd.z();
#ifdef POINT_RAW_WITH_LABEL
						pVisRaw.label = pRaw.label;
#endif
#ifdef POINT_REC_WITH_LABEL
						pVisRec.label = pRec.segLabel;
#endif
					}

					//
#ifdef POINT_RAW_WITH_NORMAL
					pVisRaw.normal_x += pRaw.normal_x;
					pVisRaw.normal_y += pRaw.normal_y;
					pVisRaw.normal_z += pRaw.normal_z;
					pVisRaw.curvature += pRaw.curvature;
#endif
#ifdef POINT_REC_WITH_NORMAL
					pVisRec.normal_x += pRec.normal_x;
					pVisRec.normal_y += pRec.normal_y;
					pVisRec.normal_z += pRec.normal_z;
					pVisRec.curvature += pRec.curvature;
#endif

					//
#ifdef POINT_RAW_WITH_RGB
					pVisRawRGB.r += (float)pRaw.r;
					pVisRawRGB.g += (float)pRaw.g;
					pVisRawRGB.b += (float)pRaw.b;
#endif
#ifdef POINT_REC_WITH_RGB
					pVisRecRGB.r += (float)pRec.r;
					pVisRecRGB.g += (float)pRec.g;
					pVisRecRGB.b += (float)pRec.b;
#endif

					//
#ifdef POINT_RAW_WITH_INTENSITY
					pVisRaw.intensity += pRaw.intensity;
#endif
#ifdef POINT_REC_WITH_INTENSITY
					pVisRec.intensity += pRec.intensity;
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
#ifdef POINT_RAW_WITH_NORMAL
					pVisRaw.normal_x /= pVisRaw.x;
					pVisRaw.normal_y /= pVisRaw.x;
					pVisRaw.normal_z /= pVisRaw.x;
					pVisRaw.curvature /= pVisRaw.x;
#endif

#ifdef POINT_REC_WITH_NORMAL
					pVisRec.normal_x /= pVisRec.x;
					pVisRec.normal_y /= pVisRec.x;
					pVisRec.normal_z /= pVisRec.x;
					pVisRec.curvature /= pVisRec.x;
#endif


#ifdef POINT_RAW_WITH_RGB
					pVisRaw.r = std::max(std::min(pVisRawRGB.r / pVisRaw.x, 255.0f), 0.0f);
					pVisRaw.g = std::max(std::min(pVisRawRGB.g / pVisRaw.x, 255.0f), 0.0f);
					pVisRaw.b = std::max(std::min(pVisRawRGB.b / pVisRaw.x, 255.0f), 0.0f);
#endif

#ifdef POINT_REC_WITH_RGB
					pVisRec.r = std::max(std::min(pVisRecRGB.r / pVisRec.x, 255.0f), 0.0f);
					pVisRec.g = std::max(std::min(pVisRecRGB.g / pVisRec.x, 255.0f), 0.0f);
					pVisRec.b = std::max(std::min(pVisRecRGB.b / pVisRec.x, 255.0f), 0.0f);
#endif

#ifdef POINT_RAW_WITH_INTENSITY
					pVisRaw.intensity /= pVisRaw.x;
#endif

#ifdef POINT_REC_WITH_INTENSITY
					pVisRec.intensity /= pVisRec.x;
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

#ifdef POINT_RAW_WITH_NORMAL
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

#ifdef POINT_REC_WITH_NORMAL
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


#ifdef POINT_RAW_WITH_RGB
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

#ifdef POINT_REC_WITH_RGB
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

#ifdef POINT_RAW_WITH_INTENSITY
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

#ifdef POINT_REC_WITH_INTENSITY
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

#ifdef POINT_RAW_WITH_LABEL
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_raw_Label.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromLabelField<PointMED> pcie;
				pcie.setColorMode(pcl::io::PointCloudImageExtractorFromLabelField<PointMED>::COLORS_RGB_GLASBEY);
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVisRaw, image))
					THROW_EXCEPTION("Failed to extract an image from Label field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

#ifdef POINT_REC_WITH_LABEL
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
		}
	}

	void ReconstructorPc::Load()
	{
		DumpAble::Load();
		if (boost::filesystem::exists(filePath / boost::filesystem::path("pcMED.pcd")))
			pcl::io::loadPCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED);
		if (boost::filesystem::exists(filePath / boost::filesystem::path("mesh.ply")))
			pcl::io::loadPLYFile((filePath / boost::filesystem::path("mesh.ply")).string(), *mesh);
	};

	void ReconstructorPc::Dump() const
	{
		DumpAble::Dump();
		if (pcMED->size() > 0)
			pcl::io::savePCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED, true);
		if (mesh->polygons.size() > 0)
			pcl::io::savePLYFile((filePath / boost::filesystem::path("mesh.ply")).string(), *mesh);
	};

	void ReconstructorPc::Load(const nlohmann::json& j)
	{
		if (j.find("status") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"status\"");
		status = Convert<ReconstructStatus, nlohmann::json>(j["status"]);
	}

	void ReconstructorPc::Dump(nlohmann::json& j) const
	{
		j["status"] = Convert<nlohmann::json, ReconstructStatus>(status);
	}

	bool ReconstructorPc::CheckExist() const
	{
		if (!DumpAble::CheckExist())
			return false;
		return true;
	}


	void ReconstructorPc::RecPcSegment()
	{
		PRINT_INFO("Segment - Start");

		segmenter->Process(pcMED);

		PRINT_INFO("Segment - End");
	}

	void ReconstructorPc::RecMesh()
	{
		PRINT_INFO("RecMesh - Start");

		mesher->Process(pcMED, *mesh);

		PRINT_INFO("RecMesh - End");
	}
}

