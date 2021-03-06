#include <vtkSmartPointer.h>
#include <vtkFillHolesFilter.h>
#include <vtkPolyDataNormals.h>
#include <vtkCleanPolyData.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/auto_io.h>

#include "nlohmann/json.hpp"

#include "Common/PCLUtils.h"
#include "Filter/FilterPcRemoveDuplicate.h"

#include "ReconstructorPc.h"

namespace RecRoom
{
	ReconstructorPc::ReconstructorPc(
		boost::filesystem::path filePath_,
		const CONST_PTR(ScannerPc)& scanner,
		const PTR(ContainerPcNDF)& containerPcNDF,
		const CONST_PTR(Interpolator)& fieldInterpolator,
		const CONST_PTR(MeshInterpolator)& meshFieldInterpolator,
		bool useVNN,
		float resVNN)
		: DumpAble("ReconstructorPc", filePath_), status(ReconstructStatus::ReconstructStatus_UNKNOWN), scanner(scanner), containerPcNDF(containerPcNDF), useVNN(useVNN), resVNN(resVNN),
		pcMED(new PcMED), pcSoftLabel(new PcSoftLabel), pcSegMaterial(new PcMED), pcREC(new PcREC), mesh(new Mesh),
		downSampler(nullptr),
		fieldInterpolator(fieldInterpolator),
		normalEstimator(nullptr),
		diffuseEstimator(nullptr),
		specularEstimator(nullptr),
		segmenter(nullptr),
		mesher(nullptr),
		meshOutlierRemover(nullptr),
		meshFilter(nullptr),
		meshFieldInterpolator(meshFieldInterpolator),
		meshSampler(nullptr)
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
			THROW_EXCEPTION("pcMED is not created?");
		if (!pcSoftLabel)
			THROW_EXCEPTION("pcSoftLabel is not created?");
		if (!pcSegMaterial)
			THROW_EXCEPTION("pcSegMaterial is not created?");
		if (!pcREC)
			THROW_EXCEPTION("pcREC is not created?");
		if (!mesh)
			THROW_EXCEPTION("mesh is not created?");
		if (!fieldInterpolator)
			THROW_EXCEPTION("fieldInterpolator is not created?");
		if (!meshFieldInterpolator)
			THROW_EXCEPTION("meshFieldInterpolator is not created?")
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

	void ReconstructorPc::RecPcDiffuse()
	{
		if (status & ReconstructStatus::PC_DIFFUSE)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
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
		else if (diffuseEstimator)
		{
			ImplementRecPcDiffuse();
			status = (ReconstructStatus)(status | ReconstructStatus::PC_DIFFUSE);
			Dump();
		}
		else
		{
			PRINT_WARNING("diffuseEstimator is not set, ignore it");
		}
	}

	void ReconstructorPc::RecPcSpecular()
	{
		if (status & ReconstructStatus::PC_SPECULAR)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_NORMAL) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_NORMAL is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_DIFFUSE) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_DIFFUSE is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else if (specularEstimator)
		{
			ImplementRecPcSpecular();
			status = (ReconstructStatus)(status | ReconstructStatus::PC_SPECULAR);
			Dump();
		}
		else
		{
			PRINT_WARNING("specularEstimator is not set, ignore it");
		}
	}

	void ReconstructorPc::RecPcSegment()
	{
		if (status & ReconstructStatus::PC_SEGMENT)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
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
			pcSoftLabel->clear();
			ImplementRecPcSegment();
			status = (ReconstructStatus)(status | ReconstructStatus::PC_SEGMENT);
			Dump();
		}
		else
		{
			PRINT_WARNING("segmenter is not set, ignore it");
		}
	}

	void ReconstructorPc::RecSegNDF()
	{
		if (status & ReconstructStatus::SEG_NDF)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if (containerPcNDF->NumLabel() != 0)
		{
			THROW_EXCEPTION("containerPcLF is already used");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_NORMAL) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_NORMAL is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_SEGMENT) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_SEGMENT is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_DIFFUSE) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_DIFFUSE is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_SPECULAR) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_SPECULAR is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else if (pcSoftLabel->empty())
		{
			PRINT_WARNING("pcSoftLabel is empty, ignore.");
		}
		else
		{
			ImplementRecSegNDF();
			status = (ReconstructStatus)(status | ReconstructStatus::SEG_NDF);
			Dump();
		}
	}

	void ReconstructorPc::RecSegMaterial()
	{
		if (status & ReconstructStatus::SEG_MATERIAL)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_NORMAL) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_NORMAL is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_SEGMENT) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_SEGMENT is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_DIFFUSE) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_DIFFUSE is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_SPECULAR) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_SPECULAR is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::SEG_NDF) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("SEG_NDF is not reconstructed yet, ignore.");
		}
		else if (containerPcNDF->NumLabel() == 0)
		{
			PRINT_WARNING("containerPcLF is empty, ignore.");
		}
		else
		{
			pcSegMaterial->clear();
			ImplementRecSegMaterial();
			status = (ReconstructStatus)(status | ReconstructStatus::SEG_MATERIAL);
			Dump();
		}
	}

	void ReconstructorPc::RecPcRefineSpecular()
	{
		if (status & ReconstructStatus::PC_REFINE_SPECULAR)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_NORMAL) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_NORMAL is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_SEGMENT) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_SEGMENT is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_DIFFUSE) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_DIFFUSE is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_SPECULAR) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_SPECULAR is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::SEG_NDF) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("SEG_NDF is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::SEG_MATERIAL) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("SEG_MATERIAL is not reconstructed yet, ignore.");
		}
		else if (pcMED->empty())
		{
			PRINT_WARNING("pcMED is empty, ignore.");
		}
		else if (pcSoftLabel->empty())
		{
			PRINT_WARNING("pcSoftLabel is empty, ignore.");
		}
		else if (pcSegMaterial->empty())
		{
			PRINT_WARNING("pcSegMaterial is empty, ignore.");
		}
		else
		{
			ImplementRecPcRefineSpecular();
			status = (ReconstructStatus)(status | ReconstructStatus::PC_REFINE_SPECULAR);
			Dump();
		}
	}

	void ReconstructorPc::RecMeshPreprocess()
	{
		if (status & ReconstructStatus::MESH_PREPROCESS)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
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
		else
		{
			pcREC->clear();
			ImplementRecMeshPreprocess();
			status = (ReconstructStatus)(status | ReconstructStatus::MESH_PREPROCESS);
			Dump();
		}
	}

	void ReconstructorPc::RecMesh()
	{
		if (status & ReconstructStatus::MESH)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_NORMAL) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_NORMAL is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::MESH_PREPROCESS) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("MESH_PREPROCESS is not reconstructed yet, ignore.");
		}
		else if (pcREC->empty())
		{
			PRINT_WARNING("pcREC is empty, ignore.");
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

	void ReconstructorPc::RecMeshPostprocess(float holeSize,
		int laplacianNumIter, float laplacianRelaxationFactor, float laplacianConvergence,
		bool laplacianSmoothFeatureEdge, float laplacianFeatureAngle,
		bool laplacianSmoothBoundary, float laplacianEdgeAngle)
	{
		if (status & ReconstructStatus::MESH_POSTPROCESS)
		{
			PRINT_WARNING("Aready reconstructed, ignore.");
		}
		else if ((status & ReconstructStatus::POINT_CLOUD) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("POINT_CLOUD is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::PC_NORMAL) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("PC_NORMAL is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::MESH_PREPROCESS) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("MESH_PREPROCESS is not reconstructed yet, ignore.");
		}
		else if ((status & ReconstructStatus::MESH) == ReconstructStatus::ReconstructStatus_UNKNOWN)
		{
			PRINT_WARNING("MESH is not reconstructed yet, ignore.");
		}
		else if (pcREC->empty())
		{
			PRINT_WARNING("pcREC is empty, ignore.");
		}
		else
		{
			ImplementRecMeshPostprocess(holeSize,
				laplacianNumIter, laplacianRelaxationFactor, laplacianConvergence,
				laplacianSmoothFeatureEdge, laplacianFeatureAngle,
				laplacianSmoothBoundary, laplacianEdgeAngle);
			status = (ReconstructStatus)(status | ReconstructStatus::MESH_POSTPROCESS);
			Dump();
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
		for (std::size_t segID = 0; segID < containerPcNDF->NumLabel(); ++segID)
		{
			{
				PTR(PcNDF) pcNDF = containerPcNDF->GetData(segID);
				if (pcNDF->size() > 0)
				{
					{
						std::stringstream ss;
						ss << "VisualSegmentNDFs : " << segID << ", pcSize: " << pcNDF->size();
						PRINT_INFO(ss.str().c_str());
					}

					Pc<pcl::PointXYZINormal> pcVisNDF;
					pcVisNDF.width = width;
					pcVisNDF.height = height;
					pcVisNDF.is_dense = false;
					pcVisNDF.resize(width * height);

					Pc<pcl::PointXYZINormal> pcVisRecNDF;
					pcVisRecNDF.width = width;
					pcVisRecNDF.height = height;
					pcVisRecNDF.is_dense = false;
					pcVisRecNDF.resize(width * height);
					for (Pc<pcl::PointXYZINormal>::iterator it = pcVisNDF.begin(); it != pcVisNDF.end(); ++it)
					{
						it->x = 0.0;// use as counter
						it->y = 0.0;
						it->z = 0.0;
						it->normal_x = NAN;
						it->normal_y = NAN;
						it->normal_z = NAN;
						it->intensity = 0.0;
					}

					for (Pc<pcl::PointXYZINormal>::iterator it = pcVisRecNDF.begin(); it != pcVisRecNDF.end(); ++it)
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
						pcVisNDF[index].x += it->weight;
						pcVisNDF[index].intensity += it->weight * it->intensity * 128.0f;

						if ((status & ReconstructStatus::SEG_MATERIAL) != ReconstructStatus::ReconstructStatus_UNKNOWN)
						{
							pcVisRecNDF[index].x += it->weight;
							pcVisRecNDF[index].intensity += it->weight * (*pcSegMaterial)[segID].specularAlbedo * specularEstimator->SpecularDistribution(Eigen::Vector3f(it->normal_x, it->normal_y, it->normal_z), (*pcSegMaterial)[segID].specularSharpness ) * 128.0f;
						}
					}

					{
						std::size_t index = 0;
						for (Pc<pcl::PointXYZINormal>::iterator it = pcVisNDF.begin(); it != pcVisNDF.end(); ++it)
						{
							if (it->x > 0)
							{
								it->intensity /= it->x;
								it->x = 0.0f;

								std::size_t col = index % width;
								std::size_t row = index / width;
								Eigen::Vector2d uv(
									(((double)col) + 0.5 / (double)(width - 1)) * 2.0 - 1.0,
									(1.0 - ((double)row) + 0.5 / (double)(height - 1)) * 2.0 - 1.0);

								it->normal_x = uv.x();
								it->normal_y = uv.y();
								it->normal_z = std::sqrt(1.0 - uv.x()*uv.x() - uv.y()*uv.y());
							}

							index++;
						}
					}

					if ((status & ReconstructStatus::SEG_MATERIAL) != ReconstructStatus::ReconstructStatus_UNKNOWN)
					{
						std::size_t index = 0;
						for (Pc<pcl::PointXYZINormal>::iterator it = pcVisRecNDF.begin(); it != pcVisRecNDF.end(); ++it)
						{
							if (it->x > 0)
							{
								it->intensity /= it->x;
								it->x = 0.0f;

								std::size_t col = index % width;
								std::size_t row = index / width;
								Eigen::Vector2d uv(
									(((double)col) + 0.5 / (double)(width - 1)) * 2.0 - 1.0,
									(1.0 - ((double)row) + 0.5 / (double)(height - 1)) * 2.0 - 1.0);

								it->normal_x = uv.x();
								it->normal_y = uv.y();
								it->normal_z = std::sqrt(1.0 - uv.x()*uv.x() - uv.y()*uv.y());
							}

							index++;
						}
					}


					{
						std::stringstream fileName;
						fileName << segID << "_Dir.png";

						pcl::PCLImage image;
						pcl::io::PointCloudImageExtractorFromNormalField<pcl::PointXYZINormal> pcie;
						pcie.setPaintNaNsWithBlack(true);
						if (!pcie.extract(pcVisNDF, image))
							THROW_EXCEPTION("Failed to extract an image from Normal field .");
						pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualSegmentNDFs") / boost::filesystem::path(fileName.str())).string(), image);
					}

					{
						std::stringstream fileName;
						fileName << segID << "_Intensity.png";

						pcl::PCLImage image;
						pcl::io::PointCloudImageExtractorFromIntensityField<pcl::PointXYZINormal> pcie;
						pcie.setPaintNaNsWithBlack(true);
						pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
						pcie.setScalingFactor(255.f);
						if (!pcie.extract(pcVisNDF, image))
							THROW_EXCEPTION("Failed to extract an image from Intensity field .");
						pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualSegmentNDFs") / boost::filesystem::path(fileName.str())).string(), image);
					}

					{
						std::stringstream fileName;
						fileName << segID << "_RecIntensity.png";

						pcl::PCLImage image;
						pcl::io::PointCloudImageExtractorFromIntensityField<pcl::PointXYZINormal> pcie;
						pcie.setPaintNaNsWithBlack(true);
						pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
						pcie.setScalingFactor(255.f);
						if (!pcie.extract(pcVisRecNDF, image))
							THROW_EXCEPTION("Failed to extract an image from Intensity field .");
						pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualSegmentNDFs") / boost::filesystem::path(fileName.str())).string(), image);
					}
				}
			}

			/*for (std::size_t scanID = 0; scanID < containerPcNDF->NumSerialNumber(); ++scanID)
			{
				PTR(PcNDF) pcNDF = containerPcNDF->GetData(segID, scanID);
				if (pcNDF->size() > 0)
				{
					{
						std::stringstream ss;
						ss << "VisualSegmentNDFs : " << segID << "-" << scanID << ", pcSize: " << pcNDF->size();
						PRINT_INFO(ss.str().c_str());
					}

					Pc<pcl::PointXYZINormal> pcVisNDF;
					pcVisNDF.width = width;
					pcVisNDF.height = height;
					pcVisNDF.is_dense = false;
					pcVisNDF.resize(width * height);
					for (Pc<pcl::PointXYZINormal>::iterator it = pcVisNDF.begin(); it != pcVisNDF.end(); ++it)
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
						pcVisNDF[index].x += it->weight;
						pcVisNDF[index].intensity += it->weight * it->intensity * 128.0f;
					}

					std::size_t index = 0;
					for (Pc<pcl::PointXYZINormal>::iterator it = pcVisNDF.begin(); it != pcVisNDF.end(); ++it)
					{
						if (it->x > 0)
						{
							it->intensity /= it->x;
							it->x = 0.0f;

							std::size_t col = index % width;
							std::size_t row = index / width;
							Eigen::Vector2d uv(
								(((double)col) + 0.5 / (double)(width - 1)) * 2.0 - 1.0,
								(1.0 - ((double)row) + 0.5 / (double)(height - 1)) * 2.0 - 1.0);

							it->normal_x = uv.x();
							it->normal_y = uv.y();
							it->normal_z = std::sqrt(1.0 - uv.x()*uv.x() - uv.y()*uv.y());

						}

						index++;
					}

					{
						std::stringstream fileName;
						fileName << segID << "_" << scanID << "_Dir.png";

						pcl::PCLImage image;
						pcl::io::PointCloudImageExtractorFromNormalField<pcl::PointXYZINormal> pcie;
						pcie.setPaintNaNsWithBlack(true);
						if (!pcie.extract(pcVisNDF, image))
							THROW_EXCEPTION("Failed to extract an image from Normal field .");
						pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualSegmentNDFs") / boost::filesystem::path(fileName.str())).string(), image);
					}

					{
						std::stringstream fileName;
						fileName << segID << "_" << scanID << "_Intensity.png";

						pcl::PCLImage image;
						pcl::io::PointCloudImageExtractorFromIntensityField<pcl::PointXYZINormal> pcie;
						pcie.setPaintNaNsWithBlack(true);
						pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
						pcie.setScalingFactor(255.f);
						if (!pcie.extract(pcVisNDF, image))
							THROW_EXCEPTION("Failed to extract an image from Intensity field .");
						pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualSegmentNDFs") / boost::filesystem::path(fileName.str())).string(), image);
					}
				}
			}*/
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
			Pc<pcl::PointXYZINormal> pcVis1;
			Pc<pcl::PointXYZRGBL> pcVis2;
			std::size_t width = scanner->ScanImageWidth() / 4;
			std::size_t height = scanner->ScanImageHeight() / 4;
			{
				pcVis1.width = width;
				pcVis1.height = height;
				pcVis1.is_dense = false;
				pcVis1.resize(width*height);

				pcVis2.width = width;
				pcVis2.height = height;
				pcVis2.is_dense = false;
				pcVis2.resize(width*height);

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

				// 
				fieldInterpolator->ProcessInOut(accMED, pcRec, nullptr);
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
					else if (pVisRaw.z > uvd.z())
						cloest = true;
					if (cloest)
					{
						pVisRaw.z = uvd.z();
						pVisRec.z = uvd.z();
						pVisRaw.serialNumber = pRaw.serialNumber;
						pVisRec.softLabelStart = pRec.softLabelStart;
						pVisRec.softLabelEnd = pRec.softLabelEnd;
					}

					pVisRawRGB.r += (float)pRaw.r;
					pVisRawRGB.g += (float)pRaw.g;
					pVisRawRGB.b += (float)pRaw.b;
					pVisRecRGB.r += (float)pRec.r;
					pVisRecRGB.g += (float)pRec.g;
					pVisRecRGB.b += (float)pRec.b;
					pVisRaw.intensity += pRaw.intensity;
					pVisRec.normal_x += pRec.normal_x;
					pVisRec.normal_y += pRec.normal_y;
					pVisRec.normal_z += pRec.normal_z;
					pVisRec.curvature += pRec.curvature;
					pVisRec.diffuseAlbedo += pRec.diffuseAlbedo;
					pVisRec.specularAlbedo += pRec.specularAlbedo;
					pVisRec.specularSharpness += pRec.specularSharpness;

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
					pVisRaw.r = std::max(std::min(pVisRawRGB.r / pVisRaw.x, 255.0f), 0.0f);
					pVisRaw.g = std::max(std::min(pVisRawRGB.g / pVisRaw.x, 255.0f), 0.0f);
					pVisRaw.b = std::max(std::min(pVisRawRGB.b / pVisRaw.x, 255.0f), 0.0f);
					pVisRec.r = std::max(std::min(pVisRecRGB.r / pVisRec.x, 255.0f), 0.0f);
					pVisRec.g = std::max(std::min(pVisRecRGB.g / pVisRec.x, 255.0f), 0.0f);
					pVisRec.b = std::max(std::min(pVisRecRGB.b / pVisRec.x, 255.0f), 0.0f);
					pVisRaw.intensity /= pVisRaw.x;
					pVisRec.normal_x /= pVisRec.x;
					pVisRec.normal_y /= pVisRec.x;
					pVisRec.normal_z /= pVisRec.x;
					pVisRec.curvature /= pVisRec.x;
					pVisRec.diffuseAlbedo /= pVisRec.x;
					pVisRec.specularAlbedo /= pVisRec.x;
					pVisRec.specularSharpness /= pVisRec.x;
				}
			}

			// Z
			{
				for (std::size_t px = 0; px < pcVis1.size(); ++px)
					pcVis1[px].z = pcVisRaw[px].z;
				
				std::stringstream fileName;
				fileName << it->serialNumber << "_raw_Depth.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromZField<pcl::PointXYZINormal> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
				if (!pcie.extract(pcVis1, image))
					THROW_EXCEPTION("Failed to extract an image from Depth field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				for (std::size_t px = 0; px < pcVis1.size(); ++px)
					pcVis1[px].z = pcVisRec[px].z;

				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_Depth.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromZField<pcl::PointXYZINormal> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
				if (!pcie.extract(pcVis1, image))
					THROW_EXCEPTION("Failed to extract an image from Depth field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				for (std::size_t px = 0; px < pcVis2.size(); ++px)
					pcVis2[px].rgba = pcVisRaw[px].rgba;

				std::stringstream fileName;
				fileName << it->serialNumber << "_raw_RGB.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromRGBField<pcl::PointXYZRGBL> pcie;
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVis2, image))
					THROW_EXCEPTION("Failed to extract an image from RGB field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				for (std::size_t px = 0; px < pcVis2.size(); ++px)
					pcVis2[px].rgba = pcVisRec[px].rgba;

				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_RGB.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromRGBField<pcl::PointXYZRGBL> pcie;
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVis2, image))
					THROW_EXCEPTION("Failed to extract an image from RGB field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				for (std::size_t px = 0; px < pcVis1.size(); ++px)
					pcVis1[px].intensity = pcVisRaw[px].intensity;

				std::stringstream fileName;
				fileName << it->serialNumber << "_raw_Intensity.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromIntensityField<pcl::PointXYZINormal> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
				pcie.setScalingFactor(255.f);
				if (!pcie.extract(pcVis1, image))
					THROW_EXCEPTION("Failed to extract an image from Intensity field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				for (std::size_t px = 0; px < pcVis1.size(); ++px)
				{
					pcVis1[px].normal_x = pcVisRec[px].normal_x;
					pcVis1[px].normal_y = pcVisRec[px].normal_y;
					pcVis1[px].normal_z = pcVisRec[px].normal_z;
					pcVis1[px].curvature = pcVisRec[px].curvature;
				}

				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_Normal.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromNormalField<pcl::PointXYZINormal> pcie;
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVis1, image))
					THROW_EXCEPTION("Failed to extract an image from Normal field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
			{
				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_Curvature.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromCurvatureField<pcl::PointXYZINormal> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
				if (!pcie.extract(pcVis1, image))
					THROW_EXCEPTION("Failed to extract an image from Curvature field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				for (std::size_t px = 0; px < pcVis2.size(); ++px)
					pcVis2[px].label = pcVisRaw[px].serialNumber;

				std::stringstream fileName;
				fileName << it->serialNumber << "_raw_SerialNumber.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZRGBL> pcie;
				pcie.setColorMode(pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZRGBL>::COLORS_RGB_GLASBEY);
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVis2, image))
					THROW_EXCEPTION("Failed to extract an image from Label field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				for (std::size_t px = 0; px < pcVis2.size(); ++px)
					pcVis2[px].label = pcVisRec[px].softLabelStart;

				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_SoftLabelStart.png";

				pcl::PCLImage image;
				//pcl::io::PointCloudImageExtractorFromLabelField<PointVisAtt> pcie;
				pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZRGBL> pcie;
				pcie.setColorMode(pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZRGBL>::COLORS_RGB_GLASBEY);
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVis2, image))
					THROW_EXCEPTION("Failed to extract an image from SoftLabelStart field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				for (std::size_t px = 0; px < pcVis2.size(); ++px)
					pcVis2[px].label = pcVisRec[px].softLabelEnd;

				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_SoftLabelEnd.png";

				pcl::PCLImage image;
				//pcl::io::PointCloudImageExtractorFromLabelField<PointVisAtt> pcie;
				pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZRGBL> pcie;
				pcie.setColorMode(pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZRGBL>::COLORS_RGB_GLASBEY);
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVis2, image))
					THROW_EXCEPTION("Failed to extract an image from SoftLabelEnd field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			if(pcSoftLabel->size() > 0)
			{
				for (std::size_t px = 0; px < pcVis2.size(); ++px)
					pcVis2[px].label = (*pcSoftLabel)[pcVisRec[px].softLabelStart].label;

				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_Segment.png";

				pcl::PCLImage image;
				//pcl::io::PointCloudImageExtractorFromLabelField<PointVisAtt> pcie;
				pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZRGBL> pcie;
				pcie.setColorMode(pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZRGBL>::COLORS_RGB_GLASBEY);
				pcie.setPaintNaNsWithBlack(true);
				if (!pcie.extract(pcVis2, image))
					THROW_EXCEPTION("Failed to extract an image from Segment field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				for (std::size_t px = 0; px < pcVis1.size(); ++px)
					pcVis1[px].intensity = pcVisRec[px].diffuseAlbedo;

				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_DiffuseAlbedo.png";

				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromIntensityField<pcl::PointXYZINormal> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
				pcie.setScalingFactor(255.f);
				if (!pcie.extract(pcVis1, image))
					THROW_EXCEPTION("Failed to extract an image from DiffuseAlbedo field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}

			{
				for (std::size_t px = 0; px < pcVis1.size(); ++px)
					pcVis1[px].intensity = pcVisRec[px].specularAlbedo;

				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_SpecularAlbedo.png";


				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromIntensityField<pcl::PointXYZINormal> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
				pcie.setScalingFactor(255.0);
				if (!pcie.extract(pcVis1, image))
					THROW_EXCEPTION("Failed to extract an image from SpecularAlbedo field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}


			{
				for (std::size_t px = 0; px < pcVis1.size(); ++px)
					pcVis1[px].intensity = pcVisRec[px].specularSharpness;

				std::stringstream fileName;
				fileName << it->serialNumber << "_rec_SpecularSharpness.png";


				pcl::PCLImage image;
				pcl::io::PointCloudImageExtractorFromIntensityField<pcl::PointXYZINormal> pcie;
				pcie.setPaintNaNsWithBlack(true);
				pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
				//pcie.setScalingFactor(6.0);
				if (!pcie.extract(pcVis1, image))
					THROW_EXCEPTION("Failed to extract an image from SpecularSharpness field .");
				pcl::io::savePNGFile((filePath / boost::filesystem::path("VisualRecAtts") / boost::filesystem::path(fileName.str())).string(), image);
			}
		}
	}

	struct WeightKernel
	{
		int shiftRow;
		int shiftCol;
		float weight;

		WeightKernel(int shiftRow = 0, int shiftCol = 0, float weight = 1.0f) : shiftRow(shiftRow), shiftCol(shiftCol), weight(weight) {}
	};

	void ReconstructorPc::FusionScanData(uint32_t serialNumber)
	{
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("FusionScanData")))
		{
			boost::filesystem::create_directory(filePath / boost::filesystem::path("FusionScanData"));
			PRINT_INFO("Create directory: " + (filePath / boost::filesystem::path("FusionScanData")).string());
		}

		const ScanMeta& scanMeta = scanner->getScanMeta(serialNumber);
		Eigen::Matrix4d wordTocan = scanMeta.transform.inverse();
		float eps = 0.01f;
		std::size_t width = scanner->ScanImageWidth() / 4;
		std::size_t height = scanner->ScanImageHeight() / 4;

		std::vector<WeightKernel> gaussianKernel(9);
		gaussianKernel[0] = WeightKernel(-1, -1, 1.0f / 16.0f);
		gaussianKernel[1] = WeightKernel(-1, 0, 2.0f / 16.0f);
		gaussianKernel[2] = WeightKernel(-1, 1, 1.0f / 16.0f);
		gaussianKernel[3] = WeightKernel(0, -1, 2.0f / 16.0f);
		gaussianKernel[4] = WeightKernel(0, 0, 4.0f / 16.0f);
		gaussianKernel[5] = WeightKernel(0, 1, 2.0f / 16.0f);
		gaussianKernel[6] = WeightKernel(1, -1, 1.0f / 16.0f);
		gaussianKernel[7] = WeightKernel(1, 0, 2.0f / 16.0f);
		gaussianKernel[8] = WeightKernel(1, 1, 1.0f / 16.0f);

		PcMED pcVisRaw;
		std::vector<ColorHDR> pcVisRawRGB;
		Pc<pcl::PointXYZINormal> pcVis1;
		Pc<pcl::PointXYZRGBL> pcVis2;
		{
			pcVis1.width = width;
			pcVis1.height = height;
			pcVis1.is_dense = false;
			pcVis1.resize(width*height);

			pcVis2.width = width;
			pcVis2.height = height;
			pcVis2.is_dense = false;
			pcVis2.resize(width*height);

			pcVisRaw.width = width;
			pcVisRaw.height = height;
			pcVisRaw.is_dense = false;
			pcVisRaw.resize(width*height);
			pcVisRawRGB.resize(width*height);
			for (PcMED::iterator jt = pcVisRaw.begin(); jt != pcVisRaw.end(); ++jt)
			{
				jt->x = 0.0; // use as counter
				jt->y = NAN; // use as depth
				jt->z = std::numeric_limits<float>::max(); // use as depth buffer
			}
		}

		for (std::vector<ScanMeta>::const_iterator it = scanner->getScanMetaSet()->begin(); it != scanner->getScanMetaSet()->end(); ++it)
		{
			{
				std::stringstream ss;
				ss << "FusionScanData - SetCloestDepth: " << it->serialNumber << " to " << serialNumber;
				PRINT_INFO(ss.str().c_str());
			}

			//
			PTR(PcMED) pcRaw(new PcMED);
			{
				PcRAW pcRaw_;
				scanner->LoadPcRAW(it->serialNumber, pcRaw_, false);
				pcRaw->resize(pcRaw_.size());
				for (std::size_t px = 0; px < pcRaw_.size(); ++px)
					(*pcRaw)[px] = pcRaw_[px];
			}

			//
			for (std::size_t px = 0; px < pcRaw->size(); ++px)
			{
				PointMED& pRaw = (*pcRaw)[px];

				Eigen::Vector4d xyz = wordTocan * Eigen::Vector4d(pRaw.x, pRaw.y, pRaw.z, 1.0);
				Eigen::Vector3d uvd = scanner->ToScanImageUVDepth(Eigen::Vector3d(xyz.x(), xyz.y(), xyz.z()));
				int col = uvd.x() * (width - 1);
				int row = (1.0 - uvd.y()) * (height - 1);

				//if (it->serialNumber == serialNumber)
				//	uvd.z() -= 0.1f;

				for (std::vector<WeightKernel>::const_iterator kt = gaussianKernel.begin(); kt != gaussianKernel.end(); ++kt)
				{
					int col2 = col + kt->shiftCol;
					int row2 = row + kt->shiftRow;

					if ((col2 >= 0) &&
						(col2 < width) &&
						(row2 >= 0) &&
						(row2 < height))
					{
						std::size_t index = row2 * width + col2;

						PointMED& pVisRaw = pcVisRaw[index];
						ColorHDR& pVisRawRGB = pcVisRawRGB[index];
						{
							if (pVisRaw.z > uvd.z())
								pVisRaw.z = uvd.z();
						}
					}
				}
			}
		}

		for (std::vector<ScanMeta>::const_iterator it = scanner->getScanMetaSet()->begin(); it != scanner->getScanMetaSet()->end(); ++it)
		{
			{
				std::stringstream ss;
				ss << "FusionScanData - Fusion: " << it->serialNumber << " to " << serialNumber;
				PRINT_INFO(ss.str().c_str());
			}

			//
			PTR(PcMED) pcRaw(new PcMED);
			{
				PcRAW pcRaw_;
				scanner->LoadPcRAW(it->serialNumber, pcRaw_, false);
				pcRaw->resize(pcRaw_.size());
				for (std::size_t px = 0; px < pcRaw_.size(); ++px)
					(*pcRaw)[px] = pcRaw_[px];
			}

			//
			for (std::size_t px = 0; px < pcRaw->size(); ++px)
			{
				PointMED& pRaw = (*pcRaw)[px];

				Eigen::Vector4d xyz = wordTocan * Eigen::Vector4d(pRaw.x, pRaw.y, pRaw.z, 1.0);
				Eigen::Vector3d uvd = scanner->ToScanImageUVDepth(Eigen::Vector3d(xyz.x(), xyz.y(), xyz.z()));
				int col = uvd.x() * (width - 1);
				int row = (1.0 - uvd.y()) * (height - 1);

				//if (it->serialNumber == serialNumber)
				//	uvd.z() -= 0.1f;

				for (std::vector<WeightKernel>::const_iterator kt = gaussianKernel.begin(); kt != gaussianKernel.end(); ++kt)
				{
					int col2 = col + kt->shiftCol;
					int row2 = row + kt->shiftRow;

					if ((col2 >= 0) &&
						(col2 < width) &&
						(row2 >= 0) &&
						(row2 < height))
					{

						std::size_t index = row2 * width + col2;

						PointMED& pVisRaw = pcVisRaw[index];
						ColorHDR& pVisRawRGB = pcVisRawRGB[index];
						if (uvd.z() < (pVisRaw.z + eps))
						{
							pVisRaw.x += kt->weight;
							if (!pcl_isfinite(pVisRaw.y))
								pVisRaw.y = 0.0f;
							pVisRaw.y += kt->weight * uvd.z();
							pVisRawRGB.r += kt->weight * (float)pRaw.r;
							pVisRawRGB.g += kt->weight * (float)pRaw.g;
							pVisRawRGB.b += kt->weight * (float)pRaw.b;
							pVisRaw.intensity += kt->weight * pRaw.intensity;
						}
					}
				}
			}
		}

		for (std::size_t px = 0; px < pcVisRaw.size(); ++px)
		{
			PointMED& pVisRaw = pcVisRaw[px];
			ColorHDR& pVisRawRGB = pcVisRawRGB[px];

			if (pVisRaw.x > 0)
			{
				if (pcl_isfinite(pVisRaw.y))
					pVisRaw.y /= pVisRaw.x;
				pVisRaw.r = std::max(std::min(pVisRawRGB.r / pVisRaw.x, 255.0f), 0.0f);
				pVisRaw.g = std::max(std::min(pVisRawRGB.g / pVisRaw.x, 255.0f), 0.0f);
				pVisRaw.b = std::max(std::min(pVisRawRGB.b / pVisRaw.x, 255.0f), 0.0f);
				pVisRaw.intensity /= pVisRaw.x;
			}
		}

		{
			for (std::size_t px = 0; px < pcVis1.size(); ++px)
				pcVis1[px].z = pcVisRaw[px].y;

			std::stringstream fileName;
			fileName << serialNumber << "_recScan_Depth.png";

			pcl::PCLImage image;
			pcl::io::PointCloudImageExtractorFromZField<pcl::PointXYZINormal> pcie;
			pcie.setPaintNaNsWithBlack(true);
			pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
			if (!pcie.extract(pcVis1, image))
				THROW_EXCEPTION("Failed to extract an image from Depth field .");
			pcl::io::savePNGFile((filePath / boost::filesystem::path("FusionScanData") / boost::filesystem::path(fileName.str())).string(), image);
		}

		{
			for (std::size_t px = 0; px < pcVis2.size(); ++px)
				pcVis2[px].rgba = pcVisRaw[px].rgba;
			
			std::stringstream fileName;
			fileName << serialNumber << "_recScan_RGB.png";

			pcl::PCLImage image;
			pcl::io::PointCloudImageExtractorFromRGBField<pcl::PointXYZRGBL> pcie;
			pcie.setPaintNaNsWithBlack(true);
			if (!pcie.extract(pcVis2, image))
				THROW_EXCEPTION("Failed to extract an image from RGB field .");
			pcl::io::savePNGFile((filePath / boost::filesystem::path("FusionScanData") / boost::filesystem::path(fileName.str())).string(), image);
		}

		{
			for (std::size_t px = 0; px < pcVis1.size(); ++px)
				pcVis1[px].intensity = pcVisRaw[px].intensity;

			std::stringstream fileName;
			fileName << serialNumber << "_recScan_Intensity.png";

			pcl::PCLImage image;
			pcl::io::PointCloudImageExtractorFromIntensityField<pcl::PointXYZINormal> pcie;
			pcie.setPaintNaNsWithBlack(true);
			pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
			pcie.setScalingFactor(255.f);
			if (!pcie.extract(pcVis1, image))
				THROW_EXCEPTION("Failed to extract an image from Intensity field .");
			pcl::io::savePNGFile((filePath / boost::filesystem::path("FusionScanData") / boost::filesystem::path(fileName.str())).string(), image);
		}

		{
			for (std::size_t px = 0; px < pcVis2.size(); ++px)
			{
				if (pcl_isfinite(pcVisRaw[px].y))
				{
					int d = (int)(pcVisRaw[px].y * 1000.0f); // use MM as unit
					int d1 = ((d / (256 * 256 * 256)) % 256);
					int d2 = ((d / (256 * 256)) % 256);
					int d3 = ((d / 256) % 256);
					int d4 = ((d) % 256);

					pcVis2[px].r = d2;
					pcVis2[px].g = d3;
					pcVis2[px].b = d4;
				}
				else
				{
					pcVis2[px].r = 255;
					pcVis2[px].g = 255;
					pcVis2[px].b = 255;
				}
			}

			std::stringstream fileName;
			fileName << serialNumber << "_recScan_HashDepth.png";

			pcl::PCLImage image;
			pcl::io::PointCloudImageExtractorFromRGBField<pcl::PointXYZRGBL> pcie;
			pcie.setPaintNaNsWithBlack(true);
			if (!pcie.extract(pcVis2, image))
				THROW_EXCEPTION("Failed to extract an image from RGB field .");
			pcl::io::savePNGFile((filePath / boost::filesystem::path("FusionScanData") / boost::filesystem::path(fileName.str())).string(), image);
		}
	}

	void ReconstructorPc::Load()
	{
		DumpAble::Load();
		if (boost::filesystem::exists(filePath / boost::filesystem::path("pcMED.pcd")))
			pcl::io::loadPCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED);
		if (boost::filesystem::exists(filePath / boost::filesystem::path("pcSoftLabel.pcd")))
			pcl::io::loadPCDFile((filePath / boost::filesystem::path("pcSoftLabel.pcd")).string(), *pcSoftLabel);
		if (boost::filesystem::exists(filePath / boost::filesystem::path("pcSegMaterial.pcd")))
			pcl::io::loadPCDFile((filePath / boost::filesystem::path("pcSegMaterial.pcd")).string(), *pcSegMaterial);
		if (boost::filesystem::exists(filePath / boost::filesystem::path("pcREC.pcd")))
			pcl::io::loadPCDFile((filePath / boost::filesystem::path("pcREC.pcd")).string(), *pcREC);
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
		if (pcSoftLabel->size() > 0)
			pcl::io::savePCDFile((filePath / boost::filesystem::path("pcSoftLabel.pcd")).string(), *pcSoftLabel, true);
		if (pcSegMaterial->size() > 0)
			pcl::io::savePCDFile((filePath / boost::filesystem::path("pcSegMaterial.pcd")).string(), *pcSegMaterial, true);	
		if (pcREC->size() > 0)
			pcl::io::savePCDFile((filePath / boost::filesystem::path("pcREC.pcd")).string(), *pcREC, true);
		if (!mesh->cloud.data.empty())
		{
			SaveAsPLY((filePath / boost::filesystem::path("mesh.ply")).string(), *mesh, 5, false);
			pcl::io::savePCDFile((filePath / boost::filesystem::path("meshVertices.pcd")).string(), mesh->cloud,
				Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);
		}
	};

	void ReconstructorPc::Load(const nlohmann::json& j)
	{
		if (j.find("useVNN") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"useVNN\"");
		useVNN = j["useVNN"];

		if (j.find("resVNN") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"resVNN\"");
		resVNN = j["resVNN"];

		if (j.find("status") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"status\"");
		status = Convert<ReconstructStatus, nlohmann::json>(j["status"]);
	}

	void ReconstructorPc::Dump(nlohmann::json& j) const
	{
		j["useVNN"] = useVNN;
		j["resVNN"] = resVNN;
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
		segmenter->ProcessInOut(accMED, pcMED, nullptr, *pcSoftLabel);

		// fill
		{
			PTR(PcIndex) validFilter(new PcIndex);
			PTR(PcIndex) inValidFilter(new PcIndex);
			validFilter->reserve(pcMED->size());
			inValidFilter->reserve(pcMED->size());
			for (int px = 0; px < pcMED->size(); ++px)
			{
				if (segmenter->OutputPointValid((*pcMED)[px]))
					validFilter->push_back(px);
				else
					inValidFilter->push_back(px);
			}

			if ((validFilter->size() > 0) && (inValidFilter->size() > 0))
			{
				PTR(AccMED) validAcc(new KDTreeMED);
				validAcc->setInputCloud(pcMED, validFilter);

				PcMED temp;
				fieldInterpolator->Process(validAcc, pcMED, inValidFilter, temp);

				for (std::size_t idx = 0; idx < inValidFilter->size(); ++idx)
				{
					PointMED& tarP = (*pcMED)[(*inValidFilter)[idx]];
					PointMED& srcP = temp[idx];

					tarP.softLabelStart = srcP.softLabelStart;
					tarP.softLabelEnd = srcP.softLabelEnd;
				}
			}
		}
	}


	void ReconstructorPc::ImplementRecSegMaterial()
	{
		pcSegMaterial->resize(containerPcNDF->NumLabel());

		for (std::size_t segID = 0; segID < containerPcNDF->NumLabel(); ++segID)
		{
			PTR(PcNDF) pcNDF = containerPcNDF->GetData(segID);

			{
				std::stringstream ss;
				ss << "RecSegMaterial - Start - seg: " << segID << ", pcSize: " << pcNDF->size();
				PRINT_INFO(ss.str().c_str());
			}

			specularEstimator->SolveSpecular((*pcSegMaterial)[segID], *pcNDF);

			{
				std::stringstream ss;
				ss << "RecSegMaterial - End - seg: " << segID << ", pcSize: " << pcNDF->size();
				PRINT_INFO(ss.str().c_str());
			}
		}
	}

	void ReconstructorPc::ImplementRecPcRefineSpecular()
	{
		for (PcMED::iterator it = pcMED->begin(); it != pcMED->end(); ++it)
		{
			it->specularAlbedo = 0.0;
			it->specularSharpness = 0.0;
			float sumWeight = 0.0f;
			for (std::size_t sx = it->softLabelStart; sx < it->softLabelEnd; ++sx)
			{
				SoftLabel& softLabel = (*pcSoftLabel)[sx];
				PointMED& mat = (*pcSegMaterial)[softLabel.label];

				sumWeight += softLabel.weight;
				it->specularAlbedo += softLabel.weight * mat.specularAlbedo;
				it->specularSharpness += softLabel.weight * mat.specularSharpness;
			}
			it->specularAlbedo /= sumWeight;
			it->specularSharpness /= sumWeight;

			it->specularAlbedo *= it->diffuseAlbedo;
		}
	}

	void ReconstructorPc::ImplementRecMeshPreprocess()
	{
		pcREC->resize(pcMED->size());
		for (std::size_t px = 0; px < pcMED->size(); ++px)
			(*pcREC)[px] = (*pcMED)[px];

		PTR(AccREC) accREC(new KDTreeREC);
		accREC->setInputCloud(pcREC);

		if (meshOutlierRemover)
		{
			PTR(PcIndex) filterREC(new PcIndex);
			meshOutlierRemover->Process(accREC, pcREC, nullptr, *filterREC);
			PTR(PcREC) pcREC2(new PcREC);
			pcl::ExtractIndices<PointREC> extract;
			extract.setInputCloud(pcREC);
			extract.setIndices(filterREC);
			extract.setNegative(false);
			extract.filter(*pcREC2);

			PTR(AccREC) accREC2(new KDTreeREC);
			accREC2->setInputCloud(pcREC2);
			pcREC = pcREC2;
			accREC = accREC2;
		}

		if (meshSampler)
		{
			PTR(PcREC) pcREC2(new PcREC);
			meshSampler->Process(accREC, pcREC, nullptr, *pcREC2);

			PTR(AccREC) accREC2(new KDTreeREC);
			accREC2->setInputCloud(pcREC2);
			pcREC = pcREC2;
			accREC = accREC2;
		}

		if (meshFilter)
		{
			PTR(PcIndex) filterREC(new PcIndex);
			meshFilter->Process(accREC, pcREC, nullptr, *filterREC);

			PTR(PcREC) pcREC2(new PcREC);
			pcl::ExtractIndices<PointREC> extract;
			extract.setInputCloud(pcREC);
			extract.setIndices(filterREC);
			extract.setNegative(false);
			extract.filter(*pcREC2);
			pcREC = pcREC2;
		}
	}

	void ReconstructorPc::ImplementRecMesh()
	{
		PTR(AccREC) accREC(new KDTreeREC);
		accREC->setInputCloud(pcREC);
		mesher->Process(accREC, pcREC, nullptr, *mesh);
	}

	void ReconstructorPc::ImplementRecMeshPostprocess(
		float holeSize, 
		int laplacianNumIter, float laplacianRelaxationFactor, float laplacianConvergence,
		bool laplacianSmoothFeatureEdge, float laplacianFeatureAngle,
		bool laplacianSmoothBoundary, float laplacianEdgeAngle )
	{
		//
		PTR(AccREC) accREC(new KDTreeREC);
		accREC->setInputCloud(pcREC);

		//
		PTR(Pc<PointREC>) vertexREC(new Pc<PointREC>);
		pcl::fromPCLPointCloud2(mesh->cloud, *vertexREC);

		PTR(Pc<pcl::PointNormal>) vertexPN(new Pc<pcl::PointNormal>);
		vertexPN->resize(vertexREC->size());
		for (std::size_t px = 0; px < vertexREC->size(); ++px)
		{
			pcl::PointNormal& tarP = (*vertexPN)[px];
			PointREC& srcP = (*vertexREC)[px];

			tarP.x = srcP.x;
			tarP.y = srcP.y;
			tarP.z = srcP.z;
			tarP.normal_x = srcP.normal_x;
			tarP.normal_y = srcP.normal_y;
			tarP.normal_z = srcP.normal_z;
			tarP.curvature = srcP.curvature;
		}
		pcl::toPCLPointCloud2(*vertexPN, mesh->cloud);

		if (holeSize > 0.0)
		{
			vtkSmartPointer<vtkPolyData> vtkMesh;
			pcl::VTKUtils::mesh2vtk(*mesh, vtkMesh);

			vtkSmartPointer<vtkFillHolesFilter> fillHolesFilter =
				vtkSmartPointer<vtkFillHolesFilter>::New();

			fillHolesFilter->SetInputData(vtkMesh);
			fillHolesFilter->SetHoleSize(holeSize);
			fillHolesFilter->Update();

			vtkSmartPointer<vtkPolyData> polyData = fillHolesFilter->GetOutput();

			pcl::VTKUtils::vtk2mesh(polyData, *mesh);
		}

		if (laplacianNumIter > 0)
		{
			//
			pcl::MeshSmoothingLaplacianVTK vtk;
			vtk.setInputMesh(mesh);
			vtk.setNumIter(laplacianNumIter);
			vtk.setRelaxationFactor(laplacianRelaxationFactor);
			vtk.setConvergence(laplacianConvergence);
			vtk.setFeatureEdgeSmoothing(laplacianSmoothFeatureEdge);
			vtk.setFeatureAngle(laplacianFeatureAngle);
			vtk.setBoundarySmoothing(laplacianSmoothBoundary);
			vtk.setEdgeAngle(laplacianEdgeAngle);
			vtk.process(*mesh);
		}

		//
		pcl::fromPCLPointCloud2(mesh->cloud, *vertexPN);

		//
		vertexREC->resize(vertexPN->size());
		for (std::size_t px = 0; px < vertexREC->size(); ++px)
		{
			PointREC& tarP = (*vertexREC)[px];
			pcl::PointNormal& srcP = (*vertexPN)[px];

			tarP.x = srcP.x;
			tarP.y = srcP.y;
			tarP.z = srcP.z;
			tarP.normal_x = srcP.normal_x;
			tarP.normal_y = srcP.normal_y;
			tarP.normal_z = srcP.normal_z;
			tarP.curvature = srcP.curvature;
		}

		Pc<PointREC> temp;
		meshFieldInterpolator->Process(accREC, vertexREC, nullptr, temp);

		for (std::size_t px = 0; px < temp.size(); ++px)
		{
			PointREC& tarP = (*vertexREC)[px];
			PointREC& srcP = temp[px];

			float tempX = tarP.x;
			float tempY = tarP.y;
			float tempZ = tarP.z;
			float tempNX = tarP.normal_x;
			float tempNY = tarP.normal_y;
			float tempNZ = tarP.normal_z;
			float tempNC = tarP.curvature;

			tarP = srcP;

			tarP.x = tempX;
			tarP.y = tempY;
			tarP.z = tempZ;

			if (pcl_isfinite(tempNX) && pcl_isfinite(tempNY) && pcl_isfinite(tempNZ))
			{
				if (pcl_isfinite(srcP.normal_x) && pcl_isfinite(srcP.normal_y) && pcl_isfinite(srcP.normal_z))
				{
					if ((tempNX * srcP.normal_x + tempNY * srcP.normal_y + tempNZ * srcP.normal_z) > 0)
					{
						tarP.normal_x = tempNX;
						tarP.normal_y = tempNY;
						tarP.normal_z = tempNZ;
					}
					else
					{
						tarP.normal_x = -tempNX;
						tarP.normal_y = -tempNY;
						tarP.normal_z = -tempNZ;
					}
				}
				else
				{
					tarP.normal_x = tempNX;
					tarP.normal_y = tempNY;
					tarP.normal_z = tempNZ;
				}
			}

			if (pcl_isfinite(tempNC))
				tarP.curvature = tempNC;
		}

		pcl::toPCLPointCloud2(*vertexREC, mesh->cloud);

		SyncMeshNormal(*mesh);
	}
}

