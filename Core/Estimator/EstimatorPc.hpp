#pragma once

#include "EstimatorPc.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	bool EstimatorPc<InPointType, OutPointType>::CollectScanData(
		const Pc<InPointType>& cloud,
		const PcIndex& nnIndices, const std::vector<float>& nnSqrDists,
		std::vector<ScanData>& scanDataSet) const
	{
		scanDataSet.clear();
		scanDataSet.reserve(nnIndices.size());

		//
		for (int idx = 0; idx < nnIndices.size(); ++idx)
		{
			int px = nnIndices[idx];
			ScanLaser laser;
			const InPointType& searchP = cloud[px];
			if (SearchPointValid(searchP))
			{
				if (scanner->ToScanLaser(searchP, laser))
				{
					if (ScanLaserValid(Eigen::Vector3f(searchP.normal_x, searchP.normal_y, searchP.normal_z), laser))
						scanDataSet.push_back(ScanData(laser, px, std::sqrt(nnSqrDists[idx])));
				}
			}
		}

		return static_cast<int>(scanDataSet.size()) >= minRequireNumData;
	}

	template<class InPointType, class OutPointType>
	void EstimatorPc<InPointType, OutPointType>::ImplementProcess(
		const CONST_PTR(Acc<InPointType>)& searchSurface,
		const CONST_PTR(Pc<InPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<OutPointType>& output) const
	{
		output.is_dense = true;

		std::vector<int> nnIndices;
		std::vector<float> nnSqrDists;
		std::vector<ScanData> scanDataSet;

#ifdef _OPENMP
#pragma omp parallel for private (nnIndices, nnSqrDists, scanDataSet) num_threads(numThreads)
#endif
		for (int idx = 0; idx < static_cast<int> (filter->size()); ++idx)
		{
			const InPointType& inPoint = (*input)[(*filter)[idx]];
			OutPointType& outPoint = output[idx];

			if (searchSurface->radiusSearch(inPoint, searchRadius, nnIndices, nnSqrDists) > 0)
			{
				//
				if (CollectScanData(*searchSurface->getInputCloud(), nnIndices, nnSqrDists, scanDataSet))
				{
					if (!ComputeAttribute(*searchSurface->getInputCloud(), scanDataSet, outPoint))
					{
						//PRINT_WARNING("ComputeAttribute failed");
						SetAttributeNAN(outPoint);
						output.is_dense = false;
					}
				}
				else
				{
					//PRINT_WARNING("CollectScanData failed");
					SetAttributeNAN(outPoint);
					output.is_dense = false;
				}
			}
		}
	}
}
