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
		const boost::filesystem::path& filePath,
		const boost::filesystem::path& e57FilePath,
		const boost::filesystem::path& lfFilePath,
		const PTR(ContainerPcRAW)& containerPcRAW,
		unsigned char colorThresh)
		: filePath(filePath), ScannerPcE57(e57FilePath, containerPcRAW, Scanner::BLK360), colorThresh(colorThresh)
	{
		bool createNew = false;
		if (!boost::filesystem::is_directory(filePath))
		{
			createNew = true;
		}
		else if (!boost::filesystem::exists(filePath / boost::filesystem::path("metaScanner.txt")))
		{
			createNew = true;
			PRINT_WARNING("filePath is not valid: missing ./metaScanner.txt, create new");
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
	}

	void ScannerPcBLK360::LoadMeta()
	{
		//
		std::string metaPath = (filePath / boost::filesystem::path("metaScanner.txt")).string();
		std::ifstream file(metaPath, std::ios_base::in);
		if (!file)
			THROW_EXCEPTION("Load file " + metaPath + " failed.");
		nlohmann::json j;
		file >> j;

		//
		if (j.find("colorThresh") == j.end())
			THROW_EXCEPTION("metaRAW is not valid: missing \"colorThresh\"");
		colorThresh = j["colorThresh"];

		//
		file.close();
	}

	void ScannerPcBLK360::DumpMeta() const
	{
		//
		std::string metaPath = (filePath / boost::filesystem::path("metaScanner.txt")).string();
		std::ofstream file(metaPath, std::ios_base::out);
		if (!file)
			THROW_EXCEPTION("Create file " + metaPath + " failed.");
		nlohmann::json j;

		//
		j["colorThresh"] = colorThresh;

		//
		file << j;
		file.close();
	}

	std::ostream& operator << (std::ostream& os, const ScannerPcBLK360& v)
	{
		if (v.getImageFileE57())
			RecRoom::OStreamE57NodeFormat(os, 0, v.getImageFileE57()->root());
		return os;
	}
}