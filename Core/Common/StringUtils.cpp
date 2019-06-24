#include "StringUtils.h"

namespace RecRoom
{
	std::string ToUpper(const std::string& s)
	{
		std::string rs = s;
		int shift = ((int)'A') - ((int)'a');
		for (auto& c : rs)
		{
			int ci = (int)c;
			if (ci <= ((int)'z') && ci >= ((int)'a'))
				c = (char)(ci + shift);
		}
		return rs;
	}

	int IsUnsignedInt(const std::string& s)
	{
		std::string rs;
		for (auto& c : s)
		{
			int ci = (int)c;
			if (ci != ((int)' '))
			{
				if ((ci <= ((int)'9')) && (ci >= ((int)'0')))
					rs.push_back(ci);
				else
					return -1;
			}
		}
		if (rs.size() > 0)
			return std::stoi(rs);
		return -1;
	}

	bool IsDir(boost::filesystem::path filePath, bool checkExist)
	{
		if (checkExist)
			return boost::filesystem::is_directory(filePath);
		else
			return filePath.extension().empty();
	}

	bool IsFileE57(boost::filesystem::path filePath, bool checkExist)
	{
		if (ToUpper(filePath.extension().string()) != ".E57")
			return false;
		if (checkExist)
			return boost::filesystem::exists(filePath);
		else
			return true;
	}

	bool IsFilePCD(boost::filesystem::path filePath, bool checkExist)
	{
		if (ToUpper(filePath.extension().string()) != ".PCD")
			return false;
		if (checkExist)
			return boost::filesystem::exists(filePath);
		else
			return true;
	}

	bool IsFileOCT(boost::filesystem::path filePath, bool checkExist)
	{
		if (!IsDir(filePath, checkExist))
			return false;
		if (checkExist)
			return boost::filesystem::exists(filePath / boost::filesystem::path("root.oct_idx"));
		else
			return true;
	}

	/*bool IsFileRec(boost::filesystem::path filePath, bool checkExist)
	{
		if (!IsDir(filePath, checkExist))
			return false;
		if (checkExist)
		{
			if (!IsDir(filePath / boost::filesystem::path("RAW"), checkExist))
				return false;
			if (!IsDir(filePath / boost::filesystem::path("NDF"), checkExist))
				return false;
			return boost::filesystem::exists(filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx")) &&
				boost::filesystem::exists(filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx")) &&
				boost::filesystem::exists(filePath / boost::filesystem::path("meta.txt")) &&
				boost::filesystem::exists(filePath / boost::filesystem::path("pcRec.pcd"));
		}
		else
			return true;
	}

	bool IsFileE57Rec(boost::filesystem::path filePath, bool checkExist)
	{
		if (!IsFileRec(filePath, checkExist))
			return false;
		if (checkExist)
			return true;
		else
			return true;
	}*/

	bool IsFilePLY(boost::filesystem::path filePath, bool checkExist)
	{
		if (ToUpper(filePath.extension().string()) != ".PLY")
			return false;
		if (checkExist)
			return boost::filesystem::exists(filePath);
		else
			return true;
	}
}