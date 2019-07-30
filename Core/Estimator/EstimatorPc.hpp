#pragma once

#include <thread>
#include <iostream>

#include "EstimatorPc.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	void EstimatorPc<InPointType, OutPointType>::EstimationTask(
		int id,
		void* self_,
		void* searchSurface_,
		void* input_,
		void* filter_,
		void* output_)
	{
		EstimatorPc<InPointType, OutPointType>& self = (*(EstimatorPc<InPointType, OutPointType>*)(self_));
		const Acc<InPointType>& searchSurface = (*(Acc<InPointType>*)(searchSurface_));
		const Pc<InPointType>& input = (*(Pc<InPointType>*)(input_));
		const PcIndex& filter = (*(PcIndex*)(filter_));
		Pc<OutPointType>& output = (*(Pc<OutPointType>*)(output_));

		std::vector<int> nnIndices;
		std::vector<float> nnSqrDists;
		std::vector<ScanData> scanDataSet;

		for (int idx = id; idx < static_cast<int> (filter.size()); idx += self.numThreads)
		{
			const InPointType& inPoint = input[filter[idx]];
			OutPointType& outPoint = output[idx];

			if (searchSurface.radiusSearch(inPoint, self.searchRadius, nnIndices, nnSqrDists) > 0)
			{
				//
				if (self.CollectScanData(*searchSurface.getInputCloud(), nnIndices, nnSqrDists, scanDataSet))
				{
					if (!self.ComputeAttribute(*searchSurface.getInputCloud(), scanDataSet, outPoint))
					{
						//PRINT_WARNING("ComputeAttribute failed");
						self.SetAttributeNAN(outPoint);
						output.is_dense = false;
					}
				}
				else
				{
					//PRINT_WARNING("CollectScanData failed");
					self.SetAttributeNAN(outPoint);
					output.is_dense = false;
				}
			}
		}
	}

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

#ifdef _OPENMP
		std::vector<int> nnIndices;
		std::vector<float> nnSqrDists;
		std::vector<ScanData> scanDataSet;

#pragma omp parallel for private (nnIndices, nnSqrDists, scanDataSet) num_threads(numThreads)
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
#else
		PRINT_WARNING("OPENMP is not enabled, us std thread instead")
		std::vector<std::thread> threads;
		for (int i = 0; i < numThreads; i++)
			threads.push_back(std::thread(EstimatorPc::EstimationTask, i, 
				(void*)(this),
				(void*)(&(*searchSurface)),
				(void*)(&(*input)),
				(void*)(&(*filter)),
				(void*)(&output)));
		for (auto& thread : threads)
			thread.join();
#endif
	}
}
