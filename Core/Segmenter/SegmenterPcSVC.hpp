#pragma once

#include "Common/SupervoxelClustering.h"

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
		SupervoxelClustering<PointType> super(
			voxelResolution, seedResolution,
			xyzImportance, rgbImportance, intensityImportance, normalImportance, sharpnessImportance, minSize, numIter);

		super.setInputCloud(input);

		super.setIndices(filter);

		super.Extract(output);
	}
}