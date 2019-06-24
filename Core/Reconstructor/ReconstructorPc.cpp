#include <pcl/io/pcd_io.h>


#include "ReconstructorPc.h"

namespace RecRoom
{
	ReconstructorPc::ReconstructorPc(
		boost::filesystem::path filePath,
		const CONST_PTR(ScannerPc)& scanner,
		const CONST_PTR(ContainerPcRAW)& containerPcRAW,
		const PTR(ContainerPcNDF)& containerPcNDF)
		: scanner(scanner), containerPcRAW(containerPcRAW), containerPcNDF(containerPcNDF), pcMED(new PcMED)
	{
		if (!scanner)
			THROW_EXCEPTION("scanner is not set");
		if (!containerPcRAW)
			THROW_EXCEPTION("containerPcRAW is not set");
		if (!containerPcNDF)
			THROW_EXCEPTION("containerPcNDF is not set");

		bool createNew = false;
		if (!boost::filesystem::exists(filePath))
		{
			createNew = true;
		}
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("pcMED.pcd")))
		{
			createNew = true;
			PRINT_WARNING("filePath is not valid: missing ./pcMED/, create new");
		}

		if (createNew)
		{
			if (!boost::filesystem::exists(filePath))
			{
				boost::filesystem::create_directory(filePath);
				PRINT_INFO("Create directory: " + filePath.string());
			}
			pcl::io::savePCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED, true);
		}
		else
		{
			pcl::io::loadPCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED);

		}
	}
}

