#pragma once

#include "Scan.h"

namespace RecRoom
{
	template<>
	inline Scanner Convert<Scanner, std::string>(const std::string& v)
	{
		if (v == "E57") return Scanner::E57;
		else if (v == "BLK360") return Scanner::BLK360;
		else return Scanner::Scaner_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, Scanner>(const Scanner& v)
	{
		switch (v)
		{
		case Scanner::E57: return std::string("E57"); break;
		case Scanner::BLK360: return std::string("BLK360"); break;
		default: return std::string("UNKNOWN"); break;
		}
	}
}
