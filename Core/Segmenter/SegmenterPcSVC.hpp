#pragma once

#include "Common/SupervoxelClustering.h"

#include "Filter/FilterPcRemoveNonFinite.h"

#include "SegmenterPcSVC.h"

namespace RecRoom
{
	template<class PointType>
	void SegmenterPcSVC<PointType>::ImplementProcess(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const CONST_PTR(Pc<PointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<PointType>& output) const
	{
		PTR(PcIndex) filter2(new PcIndex);
		FilterPcRemoveNonFinite<PointType> fNAN;
		fNAN.Process(searchSurface, input, filter, *filter2);

		SupervoxelClustering<PointType> super(
			voxelResolution, seedResolution,
			xyzImportance, rgbImportance, intensityImportance, normalImportance, sharpnessImportance);

		super.setInputCloud(input);

		super.setIndices(filter2);

		super.Extract(output);
	}
}