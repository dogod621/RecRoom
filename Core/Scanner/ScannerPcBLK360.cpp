#include <iomanip>

#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

#include "nlohmann/json.hpp"

#include "Common/AsyncProcess.h"

#include "ScannerPcBLK360.h"

namespace RecRoom
{
	//
	ScannerPcBLK360::ScannerPcBLK360(
		const boost::filesystem::path& filePath_,
		const boost::filesystem::path& e57FilePath,
		const boost::filesystem::path& lfFilePath,
		const PTR(ContainerPcRAW)& containerPcRAW,
		unsigned char colorThresh)
		: DumpAble("ScannerPcBLK360", filePath_), ScannerPcE57(e57FilePath, containerPcRAW, Scanner::BLK360), colorThresh(colorThresh)
	{
		if (this->CheckExist())
		{
			this->Load();
		}
		else
		{
			this->Dump();
		}
	}

	void ScannerPcBLK360::Load(const nlohmann::json& j)
	{
		if (j.find("colorThresh") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"colorThresh\"");
		colorThresh = j["colorThresh"];
	}

	void ScannerPcBLK360::Dump(nlohmann::json& j) const
	{
		j["colorThresh"] = colorThresh;
	}
}