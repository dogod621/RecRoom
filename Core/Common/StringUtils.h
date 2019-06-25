#pragma once

#include "Common.h"

namespace RecRoom
{
	std::string ToUpper(const std::string& s);
	int IsUnsignedInt(const std::string& s);
	bool IsDir(boost::filesystem::path filePath, bool checkExist = false);
	bool IsFileE57(boost::filesystem::path filePath, bool checkExist = false);
	bool IsFilePCD(boost::filesystem::path filePath, bool checkExist = false);
	bool IsFileOCT(boost::filesystem::path filePath, bool checkExist = false);
	bool IsFilePLY(boost::filesystem::path filePath, bool checkExist = false);
}

#include "StringUtils.hpp"