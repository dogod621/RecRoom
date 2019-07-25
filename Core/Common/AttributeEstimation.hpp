#pragma once

#include "AttributeEstimation.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	inline bool SurfaceEstimation<InPointType, OutPointType>::CollectScanData(
		const Pc<InPointType>& cloud, const std::size_t k, const std::vector<int>& indices, const std::vector<float>& distance,
		const InPointType& center, std::vector<ScanData>& scanDataSet) const
	{
		scanDataSet.clear();
		scanDataSet.reserve(k);
		
		//
		for (std::size_t idx = 0; idx < k; ++idx)
		{
			int px = indices[idx];
			ScanLaser laser;
			if (scanner->ToScanLaser(cloud[px], laser))
			{
				if (laser.beamFalloff > cutFalloff)
					scanDataSet.push_back(ScanData(laser, px, std::sqrt(distance[idx])));
			}
		}

		return scanDataSet.size() >= minRequireNumData;
	}

	template<class InPointType, class OutPointType>
	void SurfaceEstimation<InPointType, OutPointType>::computeFeature(PointCloudOut &output)
	{
		if (!(search_radius_ > 0.0))
			THROW_EXCEPTION("search_radius_ is not set");
		output.is_dense = true;
		if (input_->is_dense)
		{
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads(threads_)
#endif
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				const PointMED& inPoint = (*input_)[(*indices_)[idx]];
				PointMED& outPoint = output.points[idx];

				//
				std::vector<int> nnIndices(k_);
				std::vector<float> nnDists(k_);
				int k = searchForNeighbors((*indices_)[idx], search_parameter_, nnIndices, nnDists);

				//
				std::vector<ScanData> scanDataSet;
				if (CollectScanData(
					*surface_, k, nnIndices, nnDists,
					inPoint, scanDataSet))
				{
					if (!ComputeAttribute(
						*surface_, 
						inPoint, scanDataSet, outPoint))
					{
						PRINT_WARNING("ComputeAttribute failed");
						SetAttributeNAN(outPoint);
						output.is_dense = false;
					}
				}
				else
				{
					PRINT_WARNING("CollectScanData failed");
					SetAttributeNAN(outPoint);
					output.is_dense = false;
				}
			}
		}
		else
		{
#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads(threads_)
#endif
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				const PointMED& inPoint = (*input_)[(*indices_)[idx]];
				PointMED& outPoint = output.points[idx];

				if (pcl::isFinite(inPoint))
				{
					std::vector<int> nnIndices(k_);
					std::vector<float> nnDists(k_);
					int k = searchForNeighbors((*indices_)[idx], search_parameter_, nnIndices, nnDists);

					//
					std::vector<ScanData> scanDataSet;
					if (CollectScanData(
						*surface_, k, nnIndices, nnDists,
						inPoint, scanDataSet))
					{
						if (!ComputeAttribute(
							*surface_, 
							inPoint, scanDataSet, outPoint))
						{
							PRINT_WARNING("ComputeAttribute failed");
							SetAttributeNAN(outPoint);
							output.is_dense = false;
						}
					}
					else
					{
						PRINT_WARNING("CollectScanData failed");
						SetAttributeNAN(outPoint);
						output.is_dense = false;
					}
				}
				else
				{
					PRINT_WARNING("Input point contain non finite value");
					SetAttributeNAN(outPoint);
					output.is_dense = false;
				}
			}
		}
	}

	template<class InPointType, class OutPointType>
	inline bool AttributeEstimation<InPointType, OutPointType>::CollectScanData(
		const Pc<InPointType>& cloud, const std::size_t k, const std::vector<int>& indices, const std::vector<float>& distance,
		const InPointType& center, std::vector<ScanData>& scanDataSet) const
	{
		scanDataSet.clear();
		scanDataSet.reserve(k);

		//
		for (std::size_t idx = 0; idx < k; ++idx)
		{
			int px = indices[idx];
			ScanLaser laser;
			if (scanner->ToScanLaser(cloud[px], laser))
			{
				if ((laser.incidentDirection.dot(Eigen::Vector3f(center.normal_x, center.normal_y, center.normal_z)) > cutGrazing) &&
					(laser.beamFalloff > cutFalloff))
					scanDataSet.push_back(ScanData(laser, px, std::sqrt(distance[idx])));
			}
		}

		return scanDataSet.size() >= minRequireNumData;
	}
}