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


#define CMD_SPACE 25
#define PRINT_HELP(prefix, cmd, parms, info) std::cout << prefix << std::left << "-" << std::setw (CMD_SPACE) << cmd \
<< std::left << "[" << parms << "] : " << info << std::endl << std::endl;

void PrintHelp(int argc, char **argv)
{
	std::cout << "PrintHelp:" << std::endl << std::endl;

	std::cout << "Description:==============================================================================================================================================" << std::endl << std::endl;
	std::cout << "\tTest ReconstructorPcOC with ScannerPcBLK360, ContainerPcRAWOC, ContainerPcNDFOC" << std::endl << std::endl;

	std::cout << "Parmameters:==============================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "async", "uint 1", "Async size.");
		PRINT_HELP("\t", "src", "sting \"\"", "File path.");
		PRINT_HELP("\t", "e57", "sting \"\"", "Input e57 file.");
		PRINT_HELP("\t", "lf", "sting \"\"", "Input lf file.");
		PRINT_HELP("\t", "pcRAW", "sting \"\"", "ContainerPcRAWOC file path.");
		PRINT_HELP("\t", "pcLF", "sting \"\"", "ContainerPcLF file path.");
		PRINT_HELP("\t", "pcNDF", "sting \"\"", "ContainerPcNDFOC file path.");
		PRINT_HELP("\t", "res", "float 4", "Gird unit size of ContainerPcRAWOC in meters.");
		PRINT_HELP("\t", "min", "XYZ_string \"-100 -100 -100\"", "Min AABB corner of ContainerPcRAWOC in meters. For example: -min \"-100 -100 -100\".");
		PRINT_HELP("\t", "max", "XYZ_string \"100 100 100\"", "Max AABB corner of ContainerPcRAWOC in meters. For example: -max \"100 100 100\".");
		PRINT_HELP("\t", "overlap", "float 0.1", "Overlap size when doing out-of-core.");
	}

	std::cout << "==========================================================================================================================================================" << std::endl << std::endl;

}

int main(int argc, char *argv[])
{
	try
	{
		if (argc <= 1)
			PrintHelp(argc, argv);
		else
		{
			unsigned int async = 1;
			pcl::console::parse_argument(argc, argv, "-async", async);
			std::cout << "Parmameters -async: " << async << std::endl;

			std::string srcStr = "";
			pcl::console::parse_argument(argc, argv, "-src", srcStr);
			std::cout << "Parmameters -src: " << srcStr << std::endl;

			std::string e57Str = "";
			pcl::console::parse_argument(argc, argv, "-e57", e57Str);
			std::cout << "Parmameters -e57: " << e57Str << std::endl;

			std::string lfStr = "";
			pcl::console::parse_argument(argc, argv, "-lf", lfStr);
			std::cout << "Parmameters -lf: " << lfStr << std::endl;

			std::string pcRAWStr = "";
			pcl::console::parse_argument(argc, argv, "-pcRAW", pcRAWStr);
			std::cout << "Parmameters -pcRAW: " << pcRAWStr << std::endl;

			std::string pcNDFStr = "";
			pcl::console::parse_argument(argc, argv, "-pcNDF", pcNDFStr);
			std::cout << "Parmameters -pcNDF: " << pcNDFStr << std::endl;

			std::string pcLFStr = "";
			pcl::console::parse_argument(argc, argv, "-pcLF", pcLFStr);
			std::cout << "Parmameters -pcLF: " << pcLFStr << std::endl;

			double res = 4;
			pcl::console::parse_argument(argc, argv, "-res", res);
			std::cout << "Parmameters -res: " << res << std::endl;

			std::string minStr = "-100 -100 -100";
			std::string maxStr = "100 100 100";
			pcl::console::parse_argument(argc, argv, "-min", minStr);
			pcl::console::parse_argument(argc, argv, "-max", maxStr);
			double minX, minY, minZ, maxX, maxY, maxZ;
			std::stringstream(minStr) >> minX >> minY >> minZ;
			std::stringstream(maxStr) >> maxX >> maxY >> maxZ;
			Eigen::Vector3d min(minX, minY, minZ);
			Eigen::Vector3d max(maxX, maxY, maxZ);
			std::cout << "Parmameters -min: " << min << std::endl;
			std::cout << "Parmameters -max: " << max << std::endl;

			double overlap = 0.1;
			pcl::console::parse_argument(argc, argv, "-overlap", overlap);
			std::cout << "Parmameters -overlap: " << overlap << std::endl;

			std::cout << "Start: " << std::endl;

			std::cout << "Create ContainerPcRAWOC" << std::endl;
			PTR(RecRoom::ContainerPcRAWOC) containerPcRAW(
				new RecRoom::ContainerPcRAWOC(
					pcRAWStr, min, max, res, overlap));

			std::cout << "Create ContainerPcNDFOC" << std::endl;
			PTR(RecRoom::ContainerPcNDFOC) containerPcNDF(new RecRoom::ContainerPcNDFOC(pcNDFStr));

			//
			if (!pcLFStr.empty())
			{
				std::cout << "Create ContainerPcLF" << std::endl;
			}

			std::cout << "Create ScannerPcBLK360" << std::endl;

			PTR(RecRoom::ScannerPcBLK360) scanner(
				new RecRoom::ScannerPcBLK360(
					srcStr, e57Str, lfStr, containerPcRAW));

			//
			/*std::cout << "Print E57 Format:" << std::endl;

			std::cout << *scanner << std::endl;

			system("PAUSE");*/

			if (containerPcRAW->Size() == 0)
			{
				std::cout << "Perform ScannerPcBLK360::ShipPcRAWData" << std::endl;

				scanner->ShipPcRAWData(async);
			}

			//
			PTR(RecRoom::ReconstructorPcOC) reconstructor(new RecRoom::ReconstructorPcOC(srcStr, scanner, containerPcNDF));
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

	return EXIT_SUCCESS;
}