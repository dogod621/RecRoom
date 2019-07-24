#include "ScannerPc.h"

namespace RecRoom
{
	std::ostream& operator << (std::ostream& os, const ScanMeta& v)
	{
		os << v.serialNumber << "-" << v.numPoints;
		if (v.hasPointXYZ)
			os << "-P";
		if (v.hasPointNormal)
			os << "-N";
		if (v.hasPointRGB)
			os << "-C";
		if (v.hasPointI)
			os << "-I";
		os << "-" << Convert<std::string, CoordSys>(v.rawDataCoordSys);

		return os;
	}

	void ScannerPc::DoShipPcRAW() const
	{
		if (containerPcRAW->Size() != 0)
		{
			PRINT_WARNING("containerPcRAW is already used, ignore");
		}
		else
		{
			ShipPcRAW();
		}
	}

	void ScannerPc::DoShipPcLF() const
	{
		if (containerPcLF->Size() != 0)
		{
			PRINT_WARNING("containerPcLF is already used, ignore");
		}
		else
		{
			ShipPcLF();
		}
	}
}