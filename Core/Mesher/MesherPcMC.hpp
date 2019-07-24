#pragma once

#include "Common/MarchingCubesRBF.h"
#include "Common/MarchingCubesHoppe.h"

#include "MesherPcMC.h"

namespace RecRoom
{
	template<class PointType>
	void MesherPcMCRBF<PointType>::ToMesh(PTR(Acc<PointType>)& searchSurface, PTR(Pc<PointType>)& input, Mesh& output) const
	{
		MarchingCubesRBF<PointType> mc(offSurfaceEpsilon, percentageExtendGrid, isoLevel, gridRes, gridRes, gridRes);
		mc.setInputCloud(input);
		mc.setSearchMethod(searchSurface);
		mc.reconstruct(output);
	}

	template<class PointType>
	void MesherPcMCHoppe<PointType>::ToMesh(PTR(Acc<PointType>)& searchSurface, PTR(Pc<PointType>)& input, Mesh& output) const
	{
		MarchingCubesHoppe<PointType> mc(distIgnore, percentageExtendGrid, isoLevel, gridRes, gridRes, gridRes);
		mc.setInputCloud(input);
		mc.setSearchMethod(searchSurface);
		mc.reconstruct(output);
	}
}