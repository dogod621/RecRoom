#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>

#include "nlohmann/json.hpp"

#include "ReconstructorPc.h"

namespace RecRoom
{
	ReconstructorPc::ReconstructorPc(
		boost::filesystem::path filePath_,
		const CONST_PTR(ScannerPc)& scanner,
		const PTR(ContainerPcNDF)& containerPcNDF)
		: DumpAble("ReconstructorPc", filePath_), status(ReconstructStatus::ReconstructStatus_UNKNOWN), scanner(scanner), containerPcNDF(containerPcNDF), pcMED(new PcMED)
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
	}

	void ReconstructorPc::DoRecPointCloud()
	{
		//
		if (status != ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("Already exist reconstructed data, clear them.");
		}
		status = ReconstructStatus::ReconstructStatus_UNKNOWN;

		//
		pcMED->clear();

		//
		RecPointCloud();

		//
		status = (ReconstructStatus)(status | ReconstructStatus::POINT_CLOUD);
		this->Dump();
	}

	void ReconstructorPc::DoRecPcAlbedo()
	{
		if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
			THROW_EXCEPTION("pcMED is not reconstructed yet.");
		if (pcMED->empty())
			THROW_EXCEPTION("pcMED is empty.");

		//
		RecPcAlbedo();

		//
		status = (ReconstructStatus)(status | ReconstructStatus::PC_ALBEDO);
		this->Dump();
	}

	void ReconstructorPc::DoRecPcSegment()
	{
		if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
			THROW_EXCEPTION("pcMED is not reconstructed yet.");
		if (pcMED->empty())
			THROW_EXCEPTION("pcMED is empty.");

		//
		RecPcSegment();

		//
		status = (ReconstructStatus)(status | ReconstructStatus::PC_SEGMENT);
		this->Dump();
	}

	void ReconstructorPc::DoRecSegNDF()
	{
		if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
			THROW_EXCEPTION("pcMED is not reconstructed yet.");
		if ((status & ReconstructStatus::PC_SEGMENT) == ReconstructStatus::ReconstructStatus_UNKNOWN)
			THROW_EXCEPTION("pcMED segment is not reconstructed yet.");
		if (pcMED->empty())
			THROW_EXCEPTION("pcMED is empty.");

		//
		RecSegNDF();

		//
		status = (ReconstructStatus)(status | ReconstructStatus::SEG_NDF);
		this->Dump();
	}

	void ReconstructorPc::DoRecMesh()
	{
		if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
			THROW_EXCEPTION("pcMED is not reconstructed yet.");
		if (pcMED->empty())
			THROW_EXCEPTION("pcMED is empty.");

		//
		RecMesh();

		//
		status = (ReconstructStatus)(status | ReconstructStatus::MESH);
		this->Dump();
	}

	void ReconstructorPc::SynthScans()
	{
		if (!upSampler)
			THROW_EXCEPTION("upSampler is not set");
		if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
			THROW_EXCEPTION("pcMED is not reconstructed yet.");
		if (pcMED->empty())
			THROW_EXCEPTION("pcMED is empty.");

		PTR(KdTreeMED) accMED (new KdTreeMED);
		accMED->setInputCloud(pcMED);

		//
		for (std::vector<ScanMeta>::const_iterator it = scanner->getScanMetaSet()->begin(); it != scanner->getScanMetaSet()->end(); ++it)
		{
			{
				std::stringstream ss;
				ss << "SynthScans scanner: " << Convert<std::string, Scanner>(it->scanner) << ", serialNumber: " << it->serialNumber;
				PRINT_INFO(ss.str().c_str());
			}

			//
			PcVisAtt pcVisAtt;
			unsigned int width;
			unsigned int height;
			switch (it->scanner)
			{
			case Scanner::BLK360:
			{
				width = 1024;
				height = 512;
			}
			break;
			default:
				THROW_EXCEPTION("Scanner is not support.");
			}

			pcVisAtt.width = width;
			pcVisAtt.height = height;
			pcVisAtt.is_dense = false;
			pcVisAtt.resize(width*height);
			for (PcVisAtt::iterator jt = pcVisAtt.begin(); jt != pcVisAtt.end(); ++jt)
			{
				jt->x = 0.0; // use as counter
				jt->y = 0.0; 
				jt->z = NAN; // use as depth buffer
			}

			//
			PTR(PcMED) pcScan(new PcMED);
			PTR(PcMED) pcRecAtt(new PcMED);
			{
				PcRAW pcScan_;
				scanner->LoadPcRAW(it->serialNumber, pcScan_, false);

				pcScan->resize(pcScan_.size());
				for (std::size_t px = 0; px < pcScan_.size(); ++px)
					(*pcScan)[px] = pcScan_[px];
				(*pcRecAtt) = (*pcScan);
			}

			// Upsampling
			PcIndex upIdx;
			upSampler->Process(accMED, pcMED, pcRecAtt, upIdx);

			for (std::size_t px = 0; px < upIdx.size(); ++px)
			{
				if (upIdx[px] >= 0)
				{
					PointMED& tarP = (*pcRecAtt)[px];
					PointMED& srcP = (*pcMED)[upIdx[px]];

#ifdef POINT_MED_WITH_NORMAL
					tarP.normal_x = srcP.normal_x;
					tarP.normal_y = srcP.normal_y;
					tarP.normal_z = srcP.normal_z;
					tarP.curvature = srcP.curvature;
#endif

#ifdef POINT_MED_WITH_INTENSITY
					tarP.intensity = srcP.intensity;
#endif

#ifdef POINT_MED_WITH_SEGLABEL
					tarP.segLabel = srcP.segLabel;
#endif		
				}
			}
			

			//
			Eigen::Matrix4d wordToScan = it->transform.inverse();
			for (std::size_t px = 0; px < pcScan->size(); ++px)
			{
				PointMED& pScan = (*pcScan)[px];
				PointMED& pRecAtt = (*pcRecAtt)[px];

				Eigen::Vector4d xyz = wordToScan * Eigen::Vector4d(pScan.x, pScan.y, pScan.z, 1.0);
				Eigen::Vector2d uv;
				double depth;
				switch (it->scanner)
				{
				case Scanner::BLK360:
				{
					Eigen::Vector3d rae = CoodConvert<CoordSys::RAE_PE_PX_PY, CoordSys::XYZ_PX_PY_PZ>(Eigen::Vector3d(xyz.x(), xyz.y(), xyz.z()));
					uv = ToUV(ToMapping(UVMode::PANORAMA, CoordSys::RAE_PE_PX_PY), rae);
					depth = rae.x();
				}
				break;
				default:
					THROW_EXCEPTION("Scanner is not support.");
				}
				
				std::size_t col = uv.x() * (width - 1);
				std::size_t row = (1.0 - uv.y()) * (height - 1);
				std::size_t index = row * width + col;

				PointVisAtt& pVisAtt = pcVisAtt[index];
				{
					pVisAtt.x += 1;

					bool cloest = false;
					if (!std::isfinite(pVisAtt.z))
						cloest = true;
					else if (pVisAtt.z < depth)
						cloest = true;
					if(cloest)
					{
						pVisAtt.z = depth;
#ifdef POINT_MED_WITH_SEGLABEL
						pVisAtt.label = pRecAtt.segLabel;
#endif
					}

#ifdef POINT_MED_WITH_NORMAL
					pVisAtt.normal_x += pRecAtt.normal_x;
					pVisAtt.normal_y += pRecAtt.normal_y;
					pVisAtt.normal_z += pRecAtt.normal_z;
					pVisAtt.curvature += pRecAtt.curvature;
#endif


#ifdef POINT_MED_WITH_RGB
					pVisAtt.fR += (float)pScan.r;
					pVisAtt.fG += (float)pScan.g;
					pVisAtt.fB += (float)pScan.b;
#endif

#ifdef POINT_MED_WITH_INTENSITY
					pVisAtt.albedo += pRecAtt.intensity;
					pVisAtt.intensity += pScan.intensity;
#endif
				}
			}

			for (PcVisAtt::iterator jt = pcVisAtt.begin(); jt != pcVisAtt.end(); ++jt)
			{
				if (jt->x > 0)
				{
#ifdef POINT_MED_WITH_NORMAL
					jt->normal_x /= jt->x;
					jt->normal_y /= jt->x;
					jt->normal_z /= jt->x;
					jt->curvature /= jt->x;
#endif


#ifdef POINT_MED_WITH_RGB
					jt->fR /= jt->x;
					jt->fG /= jt->x;
					jt->fB /= jt->x;
#endif

#ifdef POINT_MED_WITH_INTENSITY
					jt->albedo /= jt->x;
					jt->intensity /= jt->x;
#endif
				}
			}

			// Z
			{
				std::stringstream fileName;
				fileName << "scan" << it->serialNumber << "_Depth.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromZField<PointVisAtt> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
				if (!pcie.extract(pcVisAtt, image))
					THROW_EXCEPTION("Failed to extract an image from Depth field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path(fileName.str())).string(), image);
			}

#ifdef POINT_MED_WITH_NORMAL
			// Normal
			{
				std::stringstream fileName;
				fileName << "scan" << it->serialNumber << "_Normal.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromNormalField<PointVisAtt> pcie;
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVisAtt, image))
					THROW_EXCEPTION("Failed to extract an image from Normal field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path(fileName.str())).string(), image);
			}

			// Curvature
			{
				std::stringstream fileName;
				fileName << "scan" << it->serialNumber << "_Curvature.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromCurvatureField<PointVisAtt> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
				if (!pcie.extract(pcVisAtt, image))
					THROW_EXCEPTION("Failed to extract an image from Curvature field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

#ifdef POINT_MED_WITH_RGB
			{
				for (PcVisAtt::iterator jt = pcVisAtt.begin(); jt != pcVisAtt.end(); ++jt)
				{
					jt->r = (unsigned char)std::max(std::min(jt->fR, 255.f), 0.f);
					jt->g = (unsigned char)std::max(std::min(jt->fG, 255.f), 0.f);
					jt->b = (unsigned char)std::max(std::min(jt->fB, 255.f), 0.f);
				}

				std::stringstream fileName;
				fileName << "scan" << it->serialNumber << "_RGB.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromRGBField<PointVisAtt> pcie;
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVisAtt, image))
					THROW_EXCEPTION("Failed to extract an image from RGB field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

#ifdef POINT_MED_WITH_INTENSITY
			{
				std::stringstream fileName;
				fileName << "scan" << it->serialNumber << "_Intensity.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromIntensityField<PointVisAtt> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
				pcie.setScalingFactor(255.f);
				if (!pcie.extract(pcVisAtt, image))
					THROW_EXCEPTION("Failed to extract an image from Intensity field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				for (PcVisAtt::iterator jt = pcVisAtt.begin(); jt != pcVisAtt.end(); ++jt)
				{
					jt->intensity = jt->albedo;
				}

				std::stringstream fileName;
				fileName << "scan" << it->serialNumber << "_Albedo.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromIntensityField<PointVisAtt> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
				pcie.setScalingFactor(255.f);
				if (!pcie.extract(pcVisAtt, image))
					THROW_EXCEPTION("Failed to extract an image from Intensity field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif

#ifdef POINT_MED_WITH_SEGLABEL
			{
				for (PcVisAtt::iterator jt = pcVisAtt.begin(); jt != pcVisAtt.end(); ++jt)
				{
					Color c = Convert<Color, uint32_t>(jt->label);
					jt->r = c.r;
					jt->g = c.g;
					jt->b = c.b;
				}

				std::stringstream fileName;
				fileName << "scan" << it->serialNumber << "_SegLabel.png";

				pcl::PCLImage image;
				//pcl::io::PointCloudImageExtractorFromLabelField<PointVisAtt> pcie;
				pcl::io::PointCloudImageExtractorFromRGBField<PointVisAtt> pcie;
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVisAtt, image))
					THROW_EXCEPTION("Failed to extract an image from Label field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path(fileName.str())).string(), image);
			}
#endif
		}
	}

	void ReconstructorPc::Load()
	{
		DumpAble::Load();
		if (boost::filesystem::exists(filePath / boost::filesystem::path("pcMED.pcd")))
			pcl::io::loadPCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED);
	};

	void ReconstructorPc::Dump() const
	{
		DumpAble::Dump();
		if (pcMED->size() > 0)
			pcl::io::savePCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED, true);
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
}

