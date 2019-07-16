#include <iostream>
#include <string>
#include <future>

#include <boost/filesystem.hpp>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "E57Format.h"

#include "Container/ContainerPcRAWOC.h"
#include "Container/ContainerPcNDFOC.h"
#include "Scanner/ScannerPcBLK360.h"
#include "Reconstructor/ReconstructorPcOC.h"

#include "Sampler/SamplerPcGrid.h"
#include "Sampler/SamplerPcNearest.h"
#include "Cropper/CropperPcOutlier.h"
#include "Estimator/EstimatorPcNormal.h"
#include "Estimator/EstimatorPcAlbedo.h"
#include "Segmenter/SegmenterPcSVC.h"
#include "Mesher/MesherPcMCRBF.h"

#define CMD_SPACE 25
#define PRINT_HELP(prefix, cmd, parms, info) std::cout << prefix << std::left << "-" << std::setw (CMD_SPACE) << cmd \
<< std::left << "[" << parms << "] : " << info << std::endl << std::endl;

void PrintHelp(int argc, char **argv)
{
	std::cout << "PrintHelp:" << std::endl << std::endl;

	std::cout << "Description:==============================================================================================================================================" << std::endl << std::endl;
	std::cout << "\tTest Reconstruct Scene with BLK360 Scanner" << std::endl << std::endl;

	std::cout << "Main Parmameters:=========================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "async", "uint 1", "Async size.");
		PRINT_HELP("\t", "wd", "sting \"\"", "Working directory.");
		PRINT_HELP("\t", "e57File", "sting \"\"", "Input e57 file path.");
		PRINT_HELP("\t", "lfFile", "sting \"\"", "Input light field file path.");
		PRINT_HELP("\t", "visSegmentNDFs", "", "Plot segment NDFs after reconstruction.");
		PRINT_HELP("\t", "visRecAtts", "", "Plot reconstruction result after reconstruction.");
	}

	std::cout << "ContainerPcRAWOC Parmameters:=============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "res", "float 4", "Gird unit size in meters.");
		PRINT_HELP("\t", "min", "XYZ_string \"-100 -100 -100\"", "Min AABB corner in meters. For example: -min \"-100 -100 -100\".");
		PRINT_HELP("\t", "max", "XYZ_string \"100 100 100\"", "Max AABB corner in meters. For example: -max \"100 100 100\".");
		PRINT_HELP("\t", "overlap", "float 0.1", "Overlap size in meters when doing out-of-core.");
	}

	std::cout << "ScannerPcBLK360 Parmameters:==============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "colorThresh", "uint 6", "For remove black noise of BLK360.");
	}

	std::cout << "SamplerPcGrid Parmameters:================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "voxelSize", "float 0.05", "Gird unit size in meters.");
	}

	std::cout << "CropperPcOutlier Parmameters:=============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "meanK", "int 50", "The number of points to use for mean distance estimation.");
		PRINT_HELP("\t", "stdMul", "float 1.0", "The standard deviation multiplier for the distance threshold calculation.");
	}
	
	std::cout << "EstimatorPc Parmameters:==================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "searchRadius", "float ${overlap}", "The search radius at surface.");
	}

	std::cout << "EstimatorPcAlbedo Parmameters:============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "distInterParm", "float 10.0", "Interpolation parameter that is related to distance.");
		PRINT_HELP("\t", "angleInterParm", "float 20.0", "Interpolation parameter that is related to orientation.");
		PRINT_HELP("\t", "cutFalloff", "float 0.33", "Cut-off threshold that is related to distance fall-off.");
		PRINT_HELP("\t", "cutGrazing", "float cos(pi/6)", "Cut-off threshold that is related to grazing level.");
	}

	std::cout << "==========================================================================================================================================================" << std::endl << std::endl;
	
	std::cout << "SegmenterPcSVC Parmameters:============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "voxelResolution", "float ${voxelSize}", "Gird unit size in meters.");
		PRINT_HELP("\t", "seedResolution", "float ${voxelSize*50}", "Seed unit size in meters.");
		PRINT_HELP("\t", "xyzImportance", "float 0.4", "Distance importance of XYZ.");
		PRINT_HELP("\t", "normalImportance", "float 1.0", "Distance importance of normal.");
		PRINT_HELP("\t", "rgbImportance", "float 0.4", "Distance importance of RGB.");
		PRINT_HELP("\t", "intensityImportance", "float 5.0", "Distance importance of intensity.");
	}

	std::cout << "MesherPcMCRBF Parmameters:============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "offSurfaceEpsilon", "float 0.1", "");
		PRINT_HELP("\t", "percentageExtendGrid", "float 0.0", "");
		PRINT_HELP("\t", "isoLevel", "float 0.0", "");
	}

	std::cout << "==========================================================================================================================================================" << std::endl << std::endl;
}

int main(int argc, char *argv[])
{
	if (argc <= 1)
		PrintHelp(argc, argv);
	else
	{
		// Parse Main Parmameters
		unsigned int async = 1;
		pcl::console::parse_argument(argc, argv, "-async", async);
		std::cout << "Main Parmameters -async: " << async << std::endl;

		std::string wdStr = "";
		pcl::console::parse_argument(argc, argv, "-wd", wdStr);
		std::cout << "Main Parmameters -wd: " << wdStr << std::endl;
		boost::filesystem::path wd(wdStr);

		if (!boost::filesystem::exists(wd))
		{
			boost::filesystem::create_directory(wd);
		}

		std::string e57File = "";
		pcl::console::parse_argument(argc, argv, "-e57File", e57File);
		std::cout << "Main Parmameters -e57File: " << e57File << std::endl;

		std::string lfFile = "";
		pcl::console::parse_argument(argc, argv, "-lfFile", lfFile);
		std::cout << "Main Parmameters -lfFile: " << lfFile << std::endl;

		// Parse ContainerPcRAWOC Parmameters
		double res = 4;
		pcl::console::parse_argument(argc, argv, "-res", res);
		std::cout << "ContainerPcRAWOC Parmameters -res: " << res << std::endl;

		std::string minStr = "-100 -100 -100";
		std::string maxStr = "100 100 100";
		pcl::console::parse_argument(argc, argv, "-min", minStr);
		pcl::console::parse_argument(argc, argv, "-max", maxStr);
		double minX, minY, minZ, maxX, maxY, maxZ;
		std::stringstream(minStr) >> minX >> minY >> minZ;
		std::stringstream(maxStr) >> maxX >> maxY >> maxZ;
		Eigen::Vector3d min(minX, minY, minZ);
		Eigen::Vector3d max(maxX, maxY, maxZ);
		std::cout << "ContainerPcRAWOC Parmameters -min: " << min << std::endl;
		std::cout << "ContainerPcRAWOC Parmameters -max: " << max << std::endl;

		double overlap = 0.1;
		pcl::console::parse_argument(argc, argv, "-overlap", overlap);
		std::cout << "ContainerPcRAWOC Parmameters -overlap: " << overlap << std::endl;

		// Parse ScannerPcBLK360 Parmameters
		unsigned int colorThresh = 6;
		pcl::console::parse_argument(argc, argv, "-colorThresh", colorThresh);
		std::cout << "ScannerPcBLK360 Parmameters -colorThresh: " << colorThresh << std::endl;


		// Parse SamplerPcGrid Parmameters
		float voxelSize = 0.05;
		pcl::console::parse_argument(argc, argv, "-voxelSize", voxelSize);
		std::cout << "SamplerPcGrid Parmameters -voxelSize: " << voxelSize << std::endl;

		// Parse CropperPcOutlier Parmameters
		int meanK = 50; 
		double stdMul = 1.0;
		pcl::console::parse_argument(argc, argv, "-meanK", meanK);
		pcl::console::parse_argument(argc, argv, "-stdMul", stdMul);
		std::cout << "CropperPcOutlier Parmameters -meanK: " << meanK << std::endl;
		std::cout << "CropperPcOutlier Parmameters -stdMul: " << stdMul << std::endl;

		// Parse EstimatorPc Parmameters
		double searchRadius = overlap;
		pcl::console::parse_argument(argc, argv, "-searchRadius", searchRadius);
		std::cout << "EstimatorPcNormal -searchRadius: " << searchRadius << std::endl;
		std::cout << "EstimatorPcAlbedo -searchRadius: " << searchRadius << std::endl;

		// Parse EstimatorPcAlbedo Parmameters
		double distInterParm = 10.0;
		double angleInterParm = 20.0;
		double cutFalloff = 0.33;
		double cutGrazing = 0.86602540378;
		pcl::console::parse_argument(argc, argv, "-distInterParm", distInterParm);
		pcl::console::parse_argument(argc, argv, "-angleInterParm", angleInterParm);
		pcl::console::parse_argument(argc, argv, "-cutFalloff", cutFalloff);
		pcl::console::parse_argument(argc, argv, "-cutGrazing", cutGrazing);
		std::cout << "EstimatorPcAlbedo -distInterParm: " << distInterParm << std::endl;
		std::cout << "EstimatorPcAlbedo -angleInterParm: " << angleInterParm << std::endl;
		std::cout << "EstimatorPcAlbedo -cutFalloff: " << cutFalloff << std::endl;
		std::cout << "EstimatorPcAlbedo -cutGrazing: " << cutGrazing << std::endl;

		// Parse SegmenterPcSVC Parmameters
		float voxelResolution = voxelSize;
		float seedResolution = voxelSize * 50.f;
		float xyzImportance = 0.4f;
		float normalImportance = 1.0f;
		float rgbImportance = 0.4f;
		float intensityImportance = 5.0f;
		pcl::console::parse_argument(argc, argv, "-voxelResolution", voxelResolution);
		pcl::console::parse_argument(argc, argv, "-seedResolution", seedResolution);
		pcl::console::parse_argument(argc, argv, "-xyzImportance", xyzImportance);
		pcl::console::parse_argument(argc, argv, "-normalImportance", normalImportance);
		pcl::console::parse_argument(argc, argv, "-rgbImportance", rgbImportance);
		pcl::console::parse_argument(argc, argv, "-intensityImportance", intensityImportance);
		std::cout << "SegmenterPcSVC -voxelResolution: " << voxelResolution << std::endl;
		std::cout << "SegmenterPcSVC -seedResolution: " << seedResolution << std::endl;
		std::cout << "SegmenterPcSVC -xyzImportance: " << xyzImportance << std::endl;
		std::cout << "SegmenterPcSVC -normalImportance: " << normalImportance << std::endl;
		std::cout << "SegmenterPcSVC -rgbImportance: " << rgbImportance << std::endl;
		std::cout << "SegmenterPcSVC -intensityImportance: " << intensityImportance << std::endl;

		// Parse MesherPcMCRBF Parmameters
		float offSurfaceEpsilon = 0.1f;
		float percentageExtendGrid = 0.0f;
		float isoLevel = 0.0f;
		pcl::console::parse_argument(argc, argv, "-offSurfaceEpsilon", offSurfaceEpsilon);
		pcl::console::parse_argument(argc, argv, "-percentageExtendGrid", percentageExtendGrid);
		pcl::console::parse_argument(argc, argv, "-isoLevel", isoLevel);
		std::cout << "SegmenterPcSVC -offSurfaceEpsilon: " << offSurfaceEpsilon << std::endl;
		std::cout << "SegmenterPcSVC -percentageExtendGrid: " << percentageExtendGrid << std::endl;
		std::cout << "SegmenterPcSVC -isoLevel: " << isoLevel << std::endl;

		// Start
		try
		{
			std::cout << "Create ContainerPcRAW" << std::endl;
			PTR(RecRoom::ContainerPcRAW)
				containerPcRAW(
					new RecRoom::ContainerPcRAWOC(
						wd / boost::filesystem::path("ContainerPcRAW"),
						min, max, res, overlap));

			std::cout << "Create ContainerPcNDF" << std::endl;
			PTR(RecRoom::ContainerPcNDF)
				containerPcNDF(
					new RecRoom::ContainerPcNDFOC(
						wd / boost::filesystem::path("ContainerPcNDF")));

			std::cout << "Create ContainerPcLF" << std::endl; 
			std::cout << "Not done, skip" << std::endl; // Not done**

			std::cout << "Create ScannerPc" << std::endl;
			PTR(RecRoom::ScannerPcBLK360)
				scannerPc(
					new RecRoom::ScannerPcBLK360(
						wd / boost::filesystem::path("ScannerPc"),
						e57File, lfFile, containerPcRAW,
						colorThresh = colorThresh));
			scannerPc->setAsyncSize(async);

			std::cout << "Create ReconstructorPc" << std::endl;
			PTR(RecRoom::ReconstructorPcOC)
				reconstructorPC(
					new RecRoom::ReconstructorPcOC(
						wd / boost::filesystem::path("ReconstructorPc"),
						scannerPc, containerPcNDF));
			reconstructorPC->setAsyncSize(async);

			std::cout << "Create DownSampler" << std::endl;
			PTR(RecRoom::ResamplerPc)
				downSampler(
					new RecRoom::SamplerPcGrid(voxelSize));
			reconstructorPC->setDownSampler(downSampler);

			std::cout << "Create UpSampler" << std::endl;
			PTR(RecRoom::SamplerPc)
				upSampler(
					new RecRoom::SamplerPcNearest());
			reconstructorPC->setUpSampler(upSampler);

			std::cout << "Create OutlierRemover" << std::endl;
			std::cout << "Not used" << std::endl;
			/*PTR(RecRoom::CropperPc)
				outlierRemover(
					new RecRoom::CropperPcOutlier(meanK, stdMul));
			reconstructorPC->setOutlierRemover(outlierRemover);*/

			std::cout << "Create NormalEstimator" << std::endl;
			PTR(RecRoom::EstimatorPc)
				normalEstimator(
					new RecRoom::EstimatorPcNormal(searchRadius));
			reconstructorPC->setNormalEstimator(normalEstimator);

			std::cout << "Create AlbedoEstimator" << std::endl;
			PTR(RecRoom::EstimatorPc)
				albedoEstimator(
					new RecRoom::EstimatorPcAlbedo(
						searchRadius, scannerPc, RecRoom::LinearSolver::EIGEN_SVD, 
						distInterParm, angleInterParm, cutFalloff, cutGrazing));
			reconstructorPC->setAlbedoEstimator(albedoEstimator);

			std::cout << "Create Segmenter" << std::endl;
			PTR(RecRoom::SegmenterPc)
				segmenter(
					new RecRoom::SegmenterPcSVC(
						voxelResolution, seedResolution,
						xyzImportance, normalImportance, rgbImportance, intensityImportance));
			reconstructorPC->setSegmenter(segmenter);

			std::cout << "Create Mesher" << std::endl;
			PTR(RecRoom::MesherPc)
				mesher(
					new RecRoom::MesherPcMCRBF(offSurfaceEpsilon, percentageExtendGrid, isoLevel));
			reconstructorPC->setMesher(mesher);

			//
			std::cout << "Print scannerPc" << std::endl;
			std::cout << (*scannerPc) << std::endl;

			//
			if (containerPcRAW->Size() == 0)
			{
				std::cout << "scannerPc->ShipPcRAW()" << std::endl;

				scannerPc->DoShipPcRAW();
			}

			if ((RecRoom::ReconstructStatus)(reconstructorPC->getStatus() & RecRoom::ReconstructStatus::POINT_CLOUD) == RecRoom::ReconstructStatus::ReconstructStatus_UNKNOWN)
			{
				std::cout << "reconstructorPC->DoRecPointCloud()" << std::endl;

				reconstructorPC->DoRecPointCloud();
			}

			if ((RecRoom::ReconstructStatus)(reconstructorPC->getStatus() & RecRoom::ReconstructStatus::PC_ALBEDO) == RecRoom::ReconstructStatus::ReconstructStatus_UNKNOWN)
			{
				std::cout << "reconstructorPC->DoRecPcAlbedo()" << std::endl;

				reconstructorPC->DoRecPcAlbedo();
			}

			if ((RecRoom::ReconstructStatus)(reconstructorPC->getStatus() & RecRoom::ReconstructStatus::PC_SEGMENT) == RecRoom::ReconstructStatus::ReconstructStatus_UNKNOWN)
			{
				std::cout << "reconstructorPC->DoRecPcSegment()" << std::endl;

				reconstructorPC->DoRecPcSegment();
			}

			if ((RecRoom::ReconstructStatus)(reconstructorPC->getStatus() & RecRoom::ReconstructStatus::SEG_NDF) == RecRoom::ReconstructStatus::ReconstructStatus_UNKNOWN)
			{
				std::cout << "reconstructorPC->DoRecSegNDF()" << std::endl;

				reconstructorPC->DoRecSegNDF();
			}

			if ((RecRoom::ReconstructStatus)(reconstructorPC->getStatus() & RecRoom::ReconstructStatus::MESH) == RecRoom::ReconstructStatus::ReconstructStatus_UNKNOWN)
			{
				std::cout << "reconstructorPC->DoRecMesh()" << std::endl;

				reconstructorPC->DoRecMesh();
			}

			//
			if (pcl::console::find_argument(argc, argv, "-visSegmentNDFs"))
			{
				std::cout << "reconstructorPC->VisualSegmentNDFs()" << std::endl;
				reconstructorPC->VisualSegmentNDFs();
			}

			if (pcl::console::find_argument(argc, argv, "-visRecAtts"))
			{
				std::cout << "reconstructorPC->VisualRecAtts()" << std::endl;
				reconstructorPC->VisualRecAtts();
			}
		}
		catch (const RecRoom::exception& ex)
		{
			std::stringstream ss;
			ss << "Caught a RecRoom::exception, what=" << ex.what();
			PRINT_ERROR(ss.str().c_str());
		}
		catch (const std::future_error& ex)
		{
			std::stringstream ss;
			ss << "Caught a std::future_error, what=" << ex.what();
			PRINT_ERROR(ss.str().c_str());
		}
		catch (const pcl::PCLException& ex)
		{
			std::stringstream ss;
			ss << "Caught a pcl::PCLException, what=" << ex.what();
			PRINT_ERROR(ss.str().c_str());
		}
		catch (const e57::E57Exception& ex)
		{
			std::stringstream ss;
			ss << "Caught an e57::E57Exception, what=" << ex.what();
			PRINT_ERROR(ss.str().c_str());
		}
		catch (const std::exception& ex)
		{
			std::stringstream ss;
			ss << "Caught a std::exception, what=" << ex.what();
			PRINT_ERROR(ss.str().c_str());
		}
		catch (...)
		{
			PRINT_ERROR("Got an unknown exception");
		}
	}
	return EXIT_SUCCESS;
}