#include <pcl/io/pcd_io.h>

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

		if (CheckExist())
		{
			Load();
		}
		else
		{
			Dump();
		}

		//
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
		Dump();
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
		Dump();
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
		Dump();
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
		Dump();
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
		Dump();
	}

	void ReconstructorPc::Load()
	{ 
		DumpAble::Load(); 
		pcl::io::loadPCDFile((filePath / boost::filesystem::path("pcMED.pcd")).string(), *pcMED);
	};

	void ReconstructorPc::Dump() const
	{ 
		DumpAble::Dump(); 
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
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("pcMED.pcd")))
			return false;
		return true;
	}
}

