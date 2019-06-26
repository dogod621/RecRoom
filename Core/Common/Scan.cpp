#include "Scan.h"

namespace RecRoom
{
	std::ostream& operator << (std::ostream& os, const ScanMeta& v)
	{
		os << Convert<std::string, Scanner>(v.scanner) << "-" << v.serialNumber << "-" << v.numPoints;
		if (v.hasPointXYZ)
			os << "-XYZ";
		if (v.hasPointNormal)
			os << "-N";
		if (v.hasPointRGB)
			os << "-RGB";
		if (v.hasPointI)
			os << "-I";
		os << "-" << Convert<std::string, CoordSys>(v.rawDataCoordSys);

		return os;
	}
}
