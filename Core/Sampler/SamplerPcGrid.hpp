#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include "Common/VoxelGridFilter.h"
#include "Filter/FilterPcRemoveDuplicate.h"
#include "SamplerPcGrid.h"

namespace RecRoom
{
	template<class PointType>
	void SamplerPcGrid<PointType>::ImplementProcess(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const CONST_PTR(Pc<PointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<PointType>& output) const
	{

		{
			PRINT_INFO("VoxelGrid Sampling - Start");

			VoxelGridFilter<PointType> vf (Eigen::Vector3d(voxelSize, voxelSize, voxelSize), minAABB, maxAABB);
			vf.setInputCloud(input);

			if (filter)
				vf.setIndices(filter);

			vf.filter(output);

			std::stringstream ss;
			ss << "Sampling - End - orgPcSize: " << input->size() << ", pcSize: " << output.size();
			PRINT_INFO(ss.str());
		}

		if (tooCloseRatio >= 0.0f)
		{
			PTR(Pc<PointType>) vp(new Pc<PointType>);
			PTR(PcIndex) vi(new PcIndex);
			(*vp) = output;
			output.clear();
			PTR(AccMED) vs = PTR(AccMED)(new KDTreeMED);
			vs->setInputCloud(vp);

			PRINT_INFO("Removing Duplicate - Start");

			FilterPcRemoveDuplicate<PointType> fd (voxelSize*1.5f, voxelSize*tooCloseRatio);
			fd.Process(vs, vp, nullptr, *vi);

			std::stringstream ss;
			ss << "Removing Duplicate - End - orgPcSize: " << vp->size() << ", pcSize: " << vi->size();
			PRINT_INFO(ss.str());

			pcl::ExtractIndices<PointType> extract;
			extract.setInputCloud(vp);
			extract.setIndices(vi);
			extract.setNegative(false);
			extract.filter(output);
		}
	}
}