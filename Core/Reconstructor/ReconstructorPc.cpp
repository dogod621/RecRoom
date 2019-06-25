#include <pcl/io/pcd_io.h>

#include "nlohmann/json.hpp"

#include "ReconstructorPc.h"

namespace RecRoom
{
	ReconstructorPc::ReconstructorPc(
		boost::filesystem::path filePath_,
		const CONST_PTR(ScannerPc)& scanner,
		const CONST_PTR(ContainerPcRAW)& containerPcRAW,
		const PTR(ContainerPcNDF)& containerPcNDF)
		: filePath(filePath_), status(ReconstructStatus::ReconstructStatus_UNKNOWN), scanner(scanner), containerPcRAW(containerPcRAW), containerPcNDF(containerPcNDF), pcMED(new PcMED)
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
			PRINT_WARNING("filePath is not valid: missing ./pcMED.pcd, create new");
		}
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("metaREC.txt")))
		{
			createNew = true;
			PRINT_WARNING("filePath is not valid: missing ./metaREC.txt, create new");
		}
		if (createNew)
		{
			if (!boost::filesystem::exists(filePath))
			{
				boost::filesystem::create_directory(filePath);
				PRINT_INFO("Create directory: " + filePath.string());
			}
			
			DumpMeta();
		}
		else
		{
			LoadMeta();
		}

		if(!pcMED)
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
		DumpMeta();
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
		DumpMeta();
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
		DumpMeta();
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
		DumpMeta();
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
		DumpMeta();
	}

	void ReconstructorPc::LoadMeta()
	{
		pcl::io::loadPCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED);

		//
		std::string metaPath = (filePath / boost::filesystem::path("metaREC.txt")).string();
		std::ifstream file(metaPath, std::ios_base::in);
		if (!file)
			THROW_EXCEPTION("Load file " + metaPath + " failed.");
		nlohmann::json j;
		file >> j;

		//
		if (j.find("status") == j.end())
			THROW_EXCEPTION("metaRAW is not valid: missing \"status\"");
		status = Convert<ReconstructStatus, nlohmann::json>(j["status"]);

		//
		file.close();
	}

	void ReconstructorPc::DumpMeta() const
	{
		pcl::io::savePCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED, true);

		//
		std::string metaPath = (filePath / boost::filesystem::path("metaREC.txt")).string();
		std::ofstream file(metaPath, std::ios_base::out);
		if (!file)
			THROW_EXCEPTION("Create file " + metaPath + " failed.");
		nlohmann::json j;

		//
		j["status"] = Convert<nlohmann::json, ReconstructStatus>(status);

		//
		file << j;
		file.close();
	}
}

