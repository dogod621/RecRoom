#pragma once

#include <thread>

#include "AttributeEstimation.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	inline bool SurfaceEstimation<InPointType, OutPointType>::CollectScanData(
		const Pc<InPointType>& cloud, int k, const std::vector<int>& indices, const std::vector<float>& distance,
		const InPointType& center, std::vector<ScanData>& scanDataSet) const
	{
		scanDataSet.clear();
		scanDataSet.reserve(k);

		//
		for (int idx = 0; idx < k; ++idx)
		{
			int px = indices[idx];
			const InPointType& inP = cloud[px];
			if (pcl::isFinite(inP))
			{
				ScanLaser laser;
				if (scanner->ToScanLaser(inP, laser))
				{
					if (laser.beamFalloff > cutFalloff)
						scanDataSet.push_back(ScanData(laser, px, std::sqrt(distance[idx])));
				}
			}
		}

		return static_cast<int>(scanDataSet.size()) >= minRequireNumData;
	}

	template<class InPointType, class OutPointType>
	void EstimationTask(int id, SurfaceEstimation<InPointType, OutPointType>* self, Pc<OutPointType>* output)
	{
		std::vector<int> nnIndices(self->k_);
		std::vector<float> nnDists(self->k_);
		std::vector<ScanData> scanDataSet(self->k_);
		for (int idx = 0; idx < static_cast<int> (self->indices_->size()); ++idx)
		{
			if (id % self->numThreads == id)
			{
				const InPointType& inPoint = (*self->input_)[(*self->indices_)[idx]];
				OutPointType& outPoint = output->points[idx];

				//
				int k = self->searchForNeighbors((*self->indices_)[idx], self->search_parameter_, nnIndices, nnDists);

				//
				if (self->CollectScanData(
					*self->surface_, k, nnIndices, nnDists,
					inPoint, scanDataSet))
				{
					if (!self->ComputeAttribute(
						*self->surface_,
						inPoint, scanDataSet, outPoint))
					{
						//PRINT_WARNING("ComputeAttribute failed");
						self->SetAttributeNAN(outPoint);
						output->is_dense = false;
					}
				}
				else
				{
					//PRINT_WARNING("CollectScanData failed");
					self->SetAttributeNAN(outPoint);
					output->is_dense = false;
				}
			}
		}
	}

	template<class InPointType, class OutPointType>
	void SurfaceEstimation<InPointType, OutPointType>::computeFeature(PointCloudOut& output)
	{
		if (!(search_radius_ > 0.0))
			THROW_EXCEPTION("search_radius_ is not set");
		output.is_dense = true;

#ifdef _OPENMP
		std::vector<int> nnIndices(k_);
		std::vector<float> nnDists(k_);
		std::vector<ScanData> scanDataSet(k_);

#pragma omp parallel for shared (output) private (nnIndices, nnDists, scanDataSet) num_threads(numThreads)
		for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
		{
			const InPointType& inPoint = (*input_)[(*indices_)[idx]];
			OutPointType& outPoint = output.points[idx];

			int k = searchForNeighbors((*indices_)[idx], search_parameter_, nnIndices, nnDists);

			//
			if (CollectScanData(
				*surface_, k, nnIndices, nnDists,
				inPoint, scanDataSet))
			{
				if (!ComputeAttribute(
					*surface_,
					inPoint, scanDataSet, outPoint))
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
#else
		std::vector<std::thread> threads;
		for (int i = 0; i < numThreads; i++)
			threads.push_back(std::thread(EstimationTask<InPointType, OutPointType>, i, this, &output));
		for (auto& thread : threads)
			thread.join();
#endif
	}

	template<class InPointType, class OutPointType>
	inline bool AttributeEstimation<InPointType, OutPointType>::CollectScanData(
		const Pc<InPointType>& cloud, const int k, const std::vector<int>& indices, const std::vector<float>& distance,
		const InPointType& center, std::vector<ScanData>& scanDataSet) const
	{
		scanDataSet.clear();
		scanDataSet.reserve(k);

		//
		for (int idx = 0; idx < k; ++idx)
		{
			int px = indices[idx];
			ScanLaser laser;
			const InPointType& inP = cloud[px];
			if (pcl::isFinite(inP))
			{
				if (scanner->ToScanLaser(inP, laser))
				{
					if ((laser.incidentDirection.dot(Eigen::Vector3f(inP.normal_x, inP.normal_y, inP.normal_z)) > cutGrazing) &&
						(laser.beamFalloff > cutFalloff))
						scanDataSet.push_back(ScanData(laser, px, std::sqrt(distance[idx])));
				}
			}
		}

		return static_cast<int>(scanDataSet.size()) >= minRequireNumData;
}
}