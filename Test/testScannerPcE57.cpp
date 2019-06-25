#include <iostream>
#include <string>

#include <boost/filesystem.hpp>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "E57Format.h"

#include "Scanner/ScannerPcE57.h"
#include "Container/ContainerPcRAWOC.h"

#define CMD_SPACE 25
#define PRINT_HELP(prefix, cmd, parms, info) std::cout << prefix << std::left << "-" << std::setw (CMD_SPACE) << cmd \
<< std::left << "[" << parms << "] : " << info << std::endl << std::endl;

void PrintHelp(int argc, char **argv)
{
	std::cout << "PrintHelp:" << std::endl << std::endl;

	std::cout << "Parmameters:==============================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "e57", "sting \"\"", "Input e57 file.");
		PRINT_HELP("\t", "container", "sting \"\"", "ContainerPcRAWOC file path.");
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

		std::string e57Str = "";
		pcl::console::parse_argument(argc, argv, "-e57", e57Str);
		std::cout << "Parmameters -src: " << e57Str << std::endl;

		std::string containerStr = "";
		pcl::console::parse_argument(argc, argv, "-container", containerStr);
		std::cout << "Parmameters -container: " << containerStr << std::endl;

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

		system("PAUSE");

		std::cout << "Start: " << std::endl;

		std::cout << "Create ContainerPcRAWOC" << std::endl;
		PTR(RecRoom::ContainerPcRAWOC) container(
			new RecRoom::ContainerPcRAWOC(
				containerStr, min, max, res, overlap));

		std::cout << "Create ScannerPcE57" << std::endl;

		PTR(RecRoom::ScannerPcE57) scanner(
			new RecRoom::ScannerPcE57(
				e57Str, container));

		std::cout << "E57 Format" << *scanner << std::endl;

		std::cout << "ScannerPcE57::ShipData" << std::endl;

		scanner->ShipData();

	}
	catch (RecRoom::exception& ex)
	{
		std::stringstream ss;
		ss << "Got an RecRoom::exception, what=" << ex.what();
		PRINT_ERROR(ss.str().c_str());
	}
	catch (pcl::PCLException& ex)
	{
		std::stringstream ss;
		ss << "Got an pcl::PCLException, what=" << ex.what();
		PRINT_ERROR(ss.str().c_str());
	}
	catch (e57::E57Exception& ex)
	{
		std::stringstream ss;
		ss << "Got an e57::E57Exception, what=" << ex.what();
		PRINT_ERROR(ss.str().c_str());
	}
	catch (std::exception& ex)
	{
		std::stringstream ss;
		ss << "Got an std::exception, what=" << ex.what();
		PRINT_ERROR(ss.str().c_str());
	}
	catch (...)
	{
		PRINT_ERROR("Got an unknown exception");
	}

	return EXIT_SUCCESS;
}