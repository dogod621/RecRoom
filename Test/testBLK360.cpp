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

#include "Estimator/EstimatorPcNormal.h"
#include "Estimator/EstimatorPcAlbedo.h"
#include "Estimator/EstimatorPcNDF.h"
#include "Estimator/EstimatorPcRefineAlbedo.h"
#include "Filter/FilterPcRemoveOutlier.h"
#include "Filter/FilterPcRemoveDuplicate.h"
#include "Mesher/MesherPcMC.h"
#include "Mesher/MesherPcGP3.h"
#include "Mesher/MesherPcGP.h"
#include "Sampler/SamplerPcGrid.h"
#include "Sampler/SamplerPcMLS.h"
#include "Segmenter/SegmenterPcSVC.h"

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
		PRINT_HELP("\t", "printScannerInfo", "", "Print scanner information.");
		PRINT_HELP("\t", "recPointCloud", "", "Reconstruct point cloud.");
		PRINT_HELP("\t", "recPcNormal", "", "Reconstruct point cloud normal.");
		PRINT_HELP("\t", "recPcDiffuse", "", "Reconstruct point cloud diffuse.");
		PRINT_HELP("\t", "recPcSpecular", "", "Reconstruct point cloud specular.");
		PRINT_HELP("\t", "recPcSegment", "", "Reconstruct point cloud segment.");
		PRINT_HELP("\t", "recSegNDF", "", "Reconstruct segment NDF.");
		PRINT_HELP("\t", "recSegMaterial", "", "Reconstruct segment material.");
		PRINT_HELP("\t", "recPcRefineSpecular", "", "Reconstruct point cloud refine specular.");
		PRINT_HELP("\t", "recMeshPreprocess", "", "Reconstruct preprocessed mesh.");
		PRINT_HELP("\t", "recMesh", "", "Reconstruct mesh.");
		PRINT_HELP("\t", "recMeshPostprocess", "", "Reconstruct postprocessed mesh.");
		PRINT_HELP("\t", "visSegNDFs", "", "Plot segment NDFs after reconstruction.");
		PRINT_HELP("\t", "visRecAtts", "", "Plot reconstruction result after reconstruction.");
	}

	std::cout << "ContainerPcRAWOC Parmameters:=============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "res", "float 4", "Gird unit size in meters.");
		PRINT_HELP("\t", "min", "float[3] -100 -100 -100", "Min AABB corner in meters. For example: -min \"-100 -100 -100\".");
		PRINT_HELP("\t", "max", "float[3] 100 100 100", "Max AABB corner in meters. For example: -max \"100 100 100\".");
		PRINT_HELP("\t", "overlap", "float ${searchRadius}", "Overlap size in meters when doing out-of-core.");
	}

	std::cout << "ScannerPcBLK360 Parmameters:==============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "colorThresh", "uint 6", "For remove black noise of BLK360.");
	}

	std::cout << "SamplerPcGrid Parmameters:================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "voxelSize", "float 0.01", "Gird unit size in meters.");
	}

	std::cout << "EstimatorPc Parmameters:==================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "searchRadius", "float ${voxelSize*5}", "The search radius at surface.");
		PRINT_HELP("\t", "distInterParm", "float 0.4", "Interpolation parameter that is related to distance.");
		PRINT_HELP("\t", "angleInterParm", "float 0.6", "Interpolation parameter that is related to orientation.");
		PRINT_HELP("\t", "cutFalloff", "float 0.33", "Cut-off threshold that is related to distance fall-off.");
		PRINT_HELP("\t", "cutGrazing", "float 0.26", "Cut-off threshold that is related to grazing level.");
	}
	
	std::cout << "SegmenterPcSVC Parmameters:===============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "voxelResolution", "float ${voxelSize}", "Gird unit size in meters.");
		PRINT_HELP("\t", "seedResolution", "float ${voxelSize*50}", "Seed unit size in meters.");
		PRINT_HELP("\t", "xyzImportance", "float 0.4", "Distance importance of XYZ.");
		PRINT_HELP("\t", "rgbImportance", "float 0.4", "Distance importance of RGB.");
		PRINT_HELP("\t", "normalImportance", "float 1.0", "Distance importance of normal.");
		PRINT_HELP("\t", "diffuseAlbedoImportance", "float 5.0", "Distance importance of diffuseAlbedoImportance.");
		PRINT_HELP("\t", "specularSharpnessImportance", "float 5.0", "Distance importance of specularSharpnessImportance.");
		PRINT_HELP("\t", "minSize", "int 2500", "Minimum segment size.");
	}

	std::cout << "MeshOutlierRemover Parmameters:===========================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "meanK", "int 50", "The number of points to use for mean distance estimation.");
		PRINT_HELP("\t", "stdMul", "float 2.0", "The standard deviation multiplier for the distance threshold calculation.");
	}

	/*std::cout << "MesherPcMCHoppe Parmameters:============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "distIgnore", "float -1.0", "");
		PRINT_HELP("\t", "percentageExtendGrid", "float 0.0", "");
		PRINT_HELP("\t", "isoLevel", "float 0.0", "");
		PRINT_HELP("\t", "gridRes", "int 256", "");
	}*/

	std::cout << "MesherPcGP3 Parmameters:==================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "maxEdgeSize", "float ${voxelSize*0.33}", "The nearest neighbors search radius for each point and the maximum edge length.");
		PRINT_HELP("\t", "mu", "float 2.5", "The nearest neighbor distance multiplier to obtain the final search radius.");
		PRINT_HELP("\t", "maxNumNei", "int ${(maxEdgeSize/voxelSize)^3*4*M_PI}", "The maximum number of nearest neighbors accepted by searching.");
		PRINT_HELP("\t", "minAngle", "float 15.0", "The preferred minimum angle in degrees for the triangles.");
		PRINT_HELP("\t", "maxAngle", "float 120.0", "The maximum angle in degrees for the triangles.");
		PRINT_HELP("\t", "epsAngle", "float 45.0", "Maximum surface angle in degrees.");
	}

	std::cout << "MeshPostprocess Parmameters:=======================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "holeSize", "float ${maxEdgeSize}", "VTK hole fill size, set negtive to disable the process.");
		PRINT_HELP("\t", "laplacianNumIter", "int 20", "VTK laplacian smoothing, set the number of iterations for the smoothing filter.");
		PRINT_HELP("\t", "laplacianRelaxationFactor", "float 0.01", "VTK laplacian smoothing, specify the relaxation factor for Laplacian smoothing.");
		PRINT_HELP("\t", "laplacianConvergence", "float 0.0", "VTK laplacian smoothing, specify a convergence criterion for the iteration process.");
		PRINT_HELP("\t", "laplacianSmoothFeatureEdge", "int 0", "VTK laplacian smoothing option,  turn on/off smoothing along sharp interior edges.");
		PRINT_HELP("\t", "laplacianFeatureAngle", "float 45.0f", "VTK laplacian smoothing, specify the feature angle for sharp edge identification.");
		PRINT_HELP("\t", "laplacianSmoothBoundary", "int 1", "VTK laplacian smoothing option, turn on/off the smoothing of vertices on the boundary of the mesh.");
		PRINT_HELP("\t", "laplacianEdgeAngle", "float 15.0f", "VTK laplacian smoothing, specify the edge angle to control smoothing along edges (either interior or boundary).");
	}

	std::cout << "==========================================================================================================================================================" << std::endl << std::endl;
}

int main(int argc, char *argv[])
{
	if (argc <= 1)
		PrintHelp(argc, argv);
	else
	{
		// Parse SamplerPcGrid Parmameters
		float voxelSize = 0.01;
		pcl::console::parse_argument(argc, argv, "-voxelSize", voxelSize);
		std::cout << "SamplerPcGrid Parmameters -voxelSize: " << voxelSize << std::endl;

		// Parse EstimatorPc Parmameters
		double searchRadius = voxelSize*5.0;
		pcl::console::parse_argument(argc, argv, "-searchRadius", searchRadius);
		std::cout << "EstimatorPc -searchRadius: " << searchRadius << std::endl;

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

		float minX = -100.f;
		float minY = -100.f;
		float minZ = -100.f;
		float maxX = 100.f;
		float maxY = 100.f;
		float maxZ = 100.f;
		pcl::console::parse_3x_arguments(argc, argv, "-min", minX, minY, minZ);
		pcl::console::parse_3x_arguments(argc, argv, "-max", maxX, maxY, maxZ);
		Eigen::Vector3d min(minX, minY, minZ);
		Eigen::Vector3d max(maxX, maxY, maxZ);
		std::cout << "ContainerPcRAWOC Parmameters -min: " << min << std::endl;
		std::cout << "ContainerPcRAWOC Parmameters -max: " << max << std::endl;

		double overlap = searchRadius;
		pcl::console::parse_argument(argc, argv, "-overlap", overlap);
		std::cout << "ContainerPcRAWOC Parmameters -overlap: " << overlap << std::endl;

		// Parse ScannerPcBLK360 Parmameters
		unsigned int colorThresh = 6;
		pcl::console::parse_argument(argc, argv, "-colorThresh", colorThresh);
		std::cout << "ScannerPcBLK360 Parmameters -colorThresh: " << colorThresh << std::endl;

		// Parse EstimatorPc Parmameters
		float distInterParm = 0.4f;
		float angleInterParm = 0.6f;
		float cutFalloff = 0.33f;
		float cutGrazing = 0.26f;
		pcl::console::parse_argument(argc, argv, "-distInterParm", distInterParm);
		pcl::console::parse_argument(argc, argv, "-angleInterParm", angleInterParm);
		pcl::console::parse_argument(argc, argv, "-cutFalloff", cutFalloff);
		pcl::console::parse_argument(argc, argv, "-cutGrazing", cutGrazing);
		std::cout << "EstimatorPc -distInterParm: " << distInterParm << std::endl;
		std::cout << "EstimatorPc -angleInterParm: " << angleInterParm << std::endl;
		std::cout << "EstimatorPc -cutFalloff: " << cutFalloff << std::endl;
		std::cout << "EstimatorPc -cutGrazing: " << cutGrazing << std::endl;

		// Parse MeshOutlierRemover Parmameters
		int meanK = 50;
		double stdMul = 2.0;
		pcl::console::parse_argument(argc, argv, "-meanK", meanK);
		pcl::console::parse_argument(argc, argv, "-stdMul", stdMul);
		std::cout << "FilterPcRemoveOutlier Parmameters -meanK: " << meanK << std::endl;
		std::cout << "FilterPcRemoveOutlier Parmameters -stdMul: " << stdMul << std::endl;

		// Parse SegmenterPcSVC Parmameters
		float voxelResolution = voxelSize;
		float seedResolution = voxelSize * 50.f;
		float xyzImportance = 0.4f;
		float rgbImportance = 0.4f;
		float normalImportance = 1.0f;
		float diffuseAlbedoImportance = 5.0f;
		float specularSharpnessImportance = 5.0f;
		int minSize = 2500;
		pcl::console::parse_argument(argc, argv, "-voxelResolution", voxelResolution);
		pcl::console::parse_argument(argc, argv, "-seedResolution", seedResolution);
		pcl::console::parse_argument(argc, argv, "-xyzImportance", xyzImportance);
		pcl::console::parse_argument(argc, argv, "-rgbImportance", rgbImportance);
		pcl::console::parse_argument(argc, argv, "-normalImportance", normalImportance);
		pcl::console::parse_argument(argc, argv, "-diffuseAlbedoImportance", diffuseAlbedoImportance);
		pcl::console::parse_argument(argc, argv, "-specularSharpnessImportance", specularSharpnessImportance);
		pcl::console::parse_argument(argc, argv, "-minSize", minSize);
		std::cout << "SegmenterPcSVC -voxelResolution: " << voxelResolution << std::endl;
		std::cout << "SegmenterPcSVC -seedResolution: " << seedResolution << std::endl;
		std::cout << "SegmenterPcSVC -xyzImportance: " << xyzImportance << std::endl;
		std::cout << "SegmenterPcSVC -rgbImportance: " << rgbImportance << std::endl;
		std::cout << "SegmenterPcSVC -normalImportance: " << normalImportance << std::endl;
		std::cout << "SegmenterPcSVC -diffuseAlbedoImportance: " << diffuseAlbedoImportance << std::endl;
		std::cout << "SegmenterPcSVC -specularSharpnessImportance: " << specularSharpnessImportance << std::endl;
		std::cout << "SegmenterPcSVC -minSize: " << minSize << std::endl;

		// Parse MesherPcMCHoppe Parmameters
		/*float distIgnore = -1.0f;
		float percentageExtendGrid = 0.0f;
		float isoLevel = 0.0f;
		int gridRes = 256;
		pcl::console::parse_argument(argc, argv, "-distIgnore", distIgnore);
		pcl::console::parse_argument(argc, argv, "-percentageExtendGrid", percentageExtendGrid);
		pcl::console::parse_argument(argc, argv, "-isoLevel", isoLevel);
		pcl::console::parse_argument(argc, argv, "-gridRes", gridRes);
		std::cout << "SegmenterPcSVC -distIgnore: " << distIgnore << std::endl;
		std::cout << "SegmenterPcSVC -percentageExtendGrid: " << percentageExtendGrid << std::endl;
		std::cout << "SegmenterPcSVC -isoLevel: " << isoLevel << std::endl;
		std::cout << "SegmenterPcSVC -gridRes: " << gridRes << std::endl;*/

		double maxEdgeSize = voxelSize*0.33;
		double mu = 2.5;
		double minAngle = 15.0;
		double maxAngle = 120.0;
		double epsAngle = 45.0;

		pcl::console::parse_argument(argc, argv, "-maxEdgeSize", maxEdgeSize);
		pcl::console::parse_argument(argc, argv, "-mu", mu);
		int maxNumNei = int(maxEdgeSize / voxelSize * maxEdgeSize / voxelSize * maxEdgeSize / voxelSize * 4.0 * M_PI);
		pcl::console::parse_argument(argc, argv, "-maxNumNei", maxNumNei);
		pcl::console::parse_argument(argc, argv, "-minAngle", minAngle);
		pcl::console::parse_argument(argc, argv, "-maxAngle", maxAngle);
		pcl::console::parse_argument(argc, argv, "-epsAngle", epsAngle);
		std::cout << "MesherPcGP3 -maxEdgeSize: " << maxEdgeSize << std::endl;
		std::cout << "MesherPcGP3 -mu: " << mu << std::endl;
		std::cout << "MesherPcGP3 -maxNumNei: " << maxNumNei << std::endl;
		std::cout << "MesherPcGP3 -minAngle: " << minAngle << std::endl;
		std::cout << "MesherPcGP3 -maxAngle: " << maxAngle << std::endl;
		std::cout << "MesherPcGP3 -epsAngle: " << epsAngle << std::endl;
		minAngle *= M_PI / 180.0;
		maxAngle *= M_PI / 180.0;
		epsAngle *= M_PI / 180.0;

		// Parse MeshPostprocess Parmameters
		double holeSize = maxEdgeSize;
		int laplacianNumIter = 20;
		float laplacianRelaxationFactor = 0.01f; 
		float laplacianConvergence = 0.0;
		int laplacianSmoothFeatureEdge = 0;
		float laplacianFeatureAngle = 45.f;
		int laplacianSmoothBoundary = 1;
		float laplacianEdgeAngle = 15.f;
		pcl::console::parse_argument(argc, argv, "-holeSize", holeSize);
		pcl::console::parse_argument(argc, argv, "-laplacianNumIter", laplacianNumIter);
		pcl::console::parse_argument(argc, argv, "-laplacianRelaxationFactor", laplacianRelaxationFactor);
		pcl::console::parse_argument(argc, argv, "-laplacianConvergence", laplacianConvergence);
		pcl::console::parse_argument(argc, argv, "-laplacianSmoothFeatureEdge", laplacianSmoothFeatureEdge);
		pcl::console::parse_argument(argc, argv, "-laplacianFeatureAngle", laplacianFeatureAngle);
		pcl::console::parse_argument(argc, argv, "-laplacianSmoothBoundary", laplacianSmoothBoundary);
		pcl::console::parse_argument(argc, argv, "-laplacianEdgeAngle", laplacianEdgeAngle);
		std::cout << "MeshPostprocess -holeSize: " << holeSize << std::endl;
		std::cout << "MeshPostprocess -laplacianNumIter: " << laplacianNumIter << std::endl;
		std::cout << "MeshPostprocess -laplacianRelaxationFactor: " << laplacianRelaxationFactor << std::endl;
		std::cout << "MeshPostprocess -laplacianConvergence: " << laplacianConvergence << std::endl;
		std::cout << "MeshPostprocess -laplacianSmoothFeatureEdge: " << laplacianSmoothFeatureEdge << std::endl;
		std::cout << "MeshPostprocess -laplacianFeatureAngle: " << laplacianFeatureAngle << std::endl;
		std::cout << "MeshPostprocess -laplacianSmoothBoundary: " << laplacianSmoothBoundary << std::endl;
		std::cout << "MeshPostprocess -laplacianEdgeAngle: " << laplacianEdgeAngle << std::endl;

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

			//
			if (pcl::console::find_switch(argc, argv, "-printScannerInfo"))
			{
				std::cout << "Print scannerPc" << std::endl;
				std::cout << (*scannerPc) << std::endl;
			}

			//
			if (containerPcRAW->Size() == 0)
			{
				std::cout << "scannerPc->ShipPcRAW()" << std::endl;

				scannerPc->DoShipPcRAW();
			}

			//
			std::cout << "Create DownSampler" << std::endl;
			{
				PTR(RecRoom::ReconstructorPcOC::Sampler)
					downSampler(
						new RecRoom::SamplerPcGrid<RecRoom::PointMED>(voxelSize, containerPcRAW->getMinAABB(), containerPcRAW->getMaxAABB()));
				reconstructorPC->setDownSampler(downSampler);
			}

			std::cout << "Create NormalEstimator" << std::endl;
			{
				PTR(RecRoom::ReconstructorPcOC::Estimator)
					normalEstimator(
						new RecRoom::EstimatorPcNormal<RecRoom::PointMED, RecRoom::PointMED>(
							scannerPc, searchRadius,
							distInterParm, cutFalloff));
				reconstructorPC->setNormalEstimator(normalEstimator);
			}

			std::cout << "Create AlbedoEstimator" << std::endl;
			{
				PTR(RecRoom::ReconstructorPcOC::Estimator)
					albedoEstimator(
						new RecRoom::EstimatorPcAlbedo<RecRoom::PointMED, RecRoom::PointMED>(
							scannerPc, searchRadius,
							3, 1, cutFalloff, cutGrazing));
				reconstructorPC->setDiffuseEstimator(albedoEstimator);
			}

			std::cout << "Create NDFEstimator" << std::endl;
			{
				PTR(RecRoom::ReconstructorPcOC::Estimator)
					ndfEstimator(
						new RecRoom::EstimatorPcSGNDF<RecRoom::PointMED, RecRoom::PointMED>(
							scannerPc, searchRadius,
							3, 1, cutFalloff, 0.0));
				reconstructorPC->setSpecularEstimator(ndfEstimator);
			}

			std::cout << "Create Segmenter" << std::endl;
			{
				PTR(RecRoom::ReconstructorPcOC::Segmenter)
					segmenter(
						new RecRoom::SegmenterPcSVC<RecRoom::PointMED>(
							voxelResolution, seedResolution,
							xyzImportance, rgbImportance, normalImportance, diffuseAlbedoImportance, specularSharpnessImportance, minSize));
				reconstructorPC->setSegmenter(segmenter);
			}

			std::cout << "Create RefineSpecularEstimator" << std::endl;
			{
				PTR(RecRoom::ReconstructorPcOC::Estimator)
					refineSpecularEstimator(
						new RecRoom::EstimatorPcRefineSGAlbedo<RecRoom::PointMED, RecRoom::PointMED>(
							scannerPc, searchRadius,
							3, 1, cutFalloff, cutGrazing));
				reconstructorPC->setRefineSpecularEstimator(refineSpecularEstimator);
			}

			std::cout << "Create MeshOutlierRemover" << std::endl;
			{
				PTR(RecRoom::ReconstructorPcOC::MeshFilter)
					meshOutlierRemover(
						new RecRoom::FilterPcRemoveOutlier<RecRoom::PointREC>(meanK, stdMul));
				reconstructorPC->setMeshOutlierRemover(meshOutlierRemover);
			}

			/*std::cout << "Create MeshSampler" << std::endl;
			{
				PTR(RecRoom::SamplerPc<RecRoom::PointREC>)
					meshSampler(
						new RecRoom::SamplerPcMLS<RecRoom::PointREC>(searchRadius * 2, 2));
				reconstructorPC->setMeshSampler(meshSampler);
			}*/


			{
				


				//
				/*PTR(RecRoom::ReconstructorPcOC::Mesher)
					mesher(
					new RecRoom::MesherPcGP3<RecRoom::PointREC>(
						maxEdgeSize, mu, maxNumNei, minAngle, maxAngle, epsAngle, false, true));*/

				//
				/*PTR(RecRoom::SamplerPc<RecRoom::PointREC>)
					distinctSampler(
						new RecRoom::SamplerPcBinaryGrid<RecRoom::PointREC>(
							voxelSize, containerPcRAW->getMinAABB(), containerPcRAW->getMaxAABB(),
							RecRoom::MorphologyOperation::DILATION, 3, 1));

				PTR(RecRoom::SamplerPc<RecRoom::PointREC>)
					preprocessSampler(
						new RecRoom::SamplerPcMLS<RecRoom::PointREC>(
							searchRadius, 5, RecRoom::MLSProjectionMethod::SIMPLE, RecRoom::MLSUpsamplingMethod::DISTINCT_CLOUD, true,
							distinctSampler ));

				PTR(RecRoom::FilterPc<RecRoom::PointREC>)
					preprocessFilter(
						new RecRoom::FilterPcRemoveDuplicate<RecRoom::PointREC>(voxelSize*0.5));

				PTR(RecRoom::ReconstructorPcOC::Mesher)
					mesher(
						new RecRoom::MesherPcGP3<RecRoom::PointREC>(
							maxEdgeSize, mu, maxNumNei, minAngle, maxAngle, epsAngle, false, true,
							preprocessSampler, preprocessFilter));*/

				//
				/*PTR(RecRoom::SamplerPc<RecRoom::PointREC>)
					preprocessSampler(
						new RecRoom::SamplerPcMLS<RecRoom::PointREC>(
							searchRadius, 5));

				PTR(RecRoom::FilterPc<RecRoom::PointREC>)
					preprocessFilter(
						new RecRoom::FilterPcRemoveDuplicate<RecRoom::PointREC>(voxelSize*0.5));*/


				/*PTR(RecRoom::SamplerPc<RecRoom::PointREC>)
					mesherPreSampler(
						new RecRoom::SamplerPcMLS<RecRoom::PointREC>(
							searchRadius * 2, 2));*/

				//reconstructorPC->setMesherPreSampler(mesherPreSampler);
			}

			std::cout << "Create Mesher" << std::endl;
			{
				PTR(RecRoom::ReconstructorPcOC::Mesher)
					mesher(
						new RecRoom::MesherPcGP3<RecRoom::PointREC>(
							maxEdgeSize, mu, maxNumNei, minAngle, maxAngle, epsAngle, true, true));

				reconstructorPC->setMesher(mesher);
			}

			//
			if (pcl::console::find_switch(argc, argv, "-recPointCloud"))
			{
				reconstructorPC->RecPointCloud();
			}

			if (pcl::console::find_switch(argc, argv, "-recPcNormal"))
			{
				reconstructorPC->RecPcNormal();
			}

			if (pcl::console::find_switch(argc, argv, "-recPcDiffuse"))
			{
				reconstructorPC->RecPcDiffuse();
			}

			if (pcl::console::find_switch(argc, argv, "-recPcSpecular"))
			{
				reconstructorPC->RecPcSpecular();
			}

			if (pcl::console::find_switch(argc, argv, "-recPcSegment"))
			{
				reconstructorPC->RecPcSegment();
			}

			if (pcl::console::find_switch(argc, argv, "-recSegNDF"))
			{
				reconstructorPC->RecSegNDF();
			}

			if (pcl::console::find_switch(argc, argv, "-recSegMaterial"))
			{
				reconstructorPC->RecSegMaterial();
			}

			if (pcl::console::find_switch(argc, argv, "-recPcRefineSpecular"))
			{
				reconstructorPC->RecPcRefineSpecular();
			}

			if (pcl::console::find_switch(argc, argv, "-recMeshPreprocess"))
			{
				reconstructorPC->RecMeshPreprocess();
			}

			if (pcl::console::find_switch(argc, argv, "-recMesh"))
			{
				reconstructorPC->RecMesh();
			}

			if(pcl::console::find_switch(argc, argv, "-recMeshPostprocess"))
			{
				reconstructorPC->RecMeshPostprocess(holeSize,
					laplacianNumIter, laplacianRelaxationFactor, laplacianConvergence,
					laplacianSmoothFeatureEdge, laplacianFeatureAngle,
					laplacianSmoothBoundary, laplacianEdgeAngle);
			}

			if (pcl::console::find_switch(argc, argv, "-visSegNDFs"))
			{
				std::cout << "reconstructorPC->VisualSegNDFs()" << std::endl;
				reconstructorPC->VisualSegNDFs();
			}

			if (pcl::console::find_switch(argc, argv, "-visRecAtts"))
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