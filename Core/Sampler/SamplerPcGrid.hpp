#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

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
			VoxelGridFilter<PointType> vf (Eigen::Vector3d(voxelSize, voxelSize, voxelSize), minAABB, maxAABB);
			vf.setInputCloud(input);
			vf.setIndices(filter);
			vf.filter(output);
		}

		if (tooCloseRatio >= 0.0f)
		{
			PTR(Pc<PointType>) vp(new Pc<PointType>);
			PTR(PcIndex) vi(new PcIndex);
			(*vp) = output;
			output.clear();
			PTR(Acc<PointType>) vs = PTR(Acc<PointType>)(new KDTree<PointType>);
			vs->setInputCloud(vp);

			FilterPcRemoveDuplicate<PointType> fd (voxelSize*tooCloseRatio);
			fd.Process(vs, vp, nullptr, *vi);

			pcl::ExtractIndices<PointType> extract;
			extract.setInputCloud(vp);
			extract.setIndices(vi);
			extract.setNegative(false);
			extract.filter(output);
		}
	}

	template<class PointType>
	void SamplerPcBinaryGrid<PointType>::ImplementProcess(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const CONST_PTR(Pc<PointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<PointType>& output) const
	{
		BinaryVoxelGrid<PointType> vf(Eigen::Vector3d(voxelSize, voxelSize, voxelSize), minAABB, maxAABB);
		vf.AddPointCloud(input, filter);

		switch (morphologyOperation)
		{
		case MorphologyOperation::MorphologyOperation_NONE:
			break;
		case MorphologyOperation::DILATION:
			vf.Dilation(kernelSize, iteration);
			break;
		case MorphologyOperation::EROSION:
			vf.Erosion(kernelSize, iteration);
			break;
		case MorphologyOperation::OPENING:
			vf.Opening(kernelSize, iteration);
			break;
		case MorphologyOperation::CLOSING:
			vf.Closing(kernelSize, iteration);
			break;
		default:
			THROW_EXCEPTION("morphologyOperation is not valid");
			break;
		}
		output = *vf.GetPointCloud();
	}
}