#pragma once

#include <thread>

#include "AttributeEstimation.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	void SurfaceEstimation<InPointType, OutPointType>::SetNumberOfThreads(unsigned int numThreads)
	{
		if (numThreads == 0)
#ifdef _OPENMP
			threads_ = omp_get_num_procs();
#else
			threads_ = std::thread::hardware_concurrency();
#endif
		else
			threads_ = numThreads;
	}

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
			ScanLaser laser;
			if (scanner->ToScanLaser(cloud[px], laser))
			{
				if (laser.beamFalloff > cutFalloff)
					scanDataSet.push_back(ScanData(laser, px, std::sqrt(distance[idx])));
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
		if (self->input_->is_dense)
		{
			for (int idx = 0; idx < static_cast<int> (self->indices_->size()); ++idx)
			{
				if (id % self->threads_ == id)
				{
					const InPointType& inPoint = (*self->input_)[(*self->indices_)[idx]];
					OutPointType& outPoint = output->points[idx];

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
							PRINT_WARNING("ComputeAttribute failed");
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
		else
		{
			for (int idx = 0; idx < static_cast<int> (self->indices_->size()); ++idx)
			{
				if (id % self->threads_ == id)
				{
					const InPointType& inPoint = (*self->input_)[(*self->indices_)[idx]];
					OutPointType& outPoint = output->points[idx];

					//
					if (pcl::isFinite(inPoint))
					{
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
								PRINT_WARNING("ComputeAttribute failed");
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
					else
					{
						PRINT_WARNING("Input point contain non finite value");
						self->SetAttributeNAN(outPoint);
						output->is_dense = false;
					}
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
		if (input_->is_dense)
		{
#pragma omp parallel for shared (output) private (nnIndices, nnDists, scanDataSet) num_threads(threads_)
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				const InPointType& inPoint = (*input_)[(*indices_)[idx]];
				OutPointType& outPoint = output.points[idx];

				//
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
						PRINT_WARNING("ComputeAttribute failed");
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
		else
		{
#pragma omp parallel for shared (output) private (nnIndices, nnDists, scanDataSet) num_threads(threads_)
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				const InPointType& inPoint = (*input_)[(*indices_)[idx]];
				OutPointType& outPoint = output.points[idx];

				if (pcl::isFinite(inPoint))
				{
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
							PRINT_WARNING("ComputeAttribute failed");
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
				else
				{
					PRINT_WARNING("Input point contain non finite value");
					SetAttributeNAN(outPoint);
					output.is_dense = false;
				}
			}
		}
#else
		std::vector<std::thread> threads;
		for (int i = 0; i < threads_; i++)
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
			if (scanner->ToScanLaser(cloud[px], laser))
			{
				if ((laser.incidentDirection.dot(Eigen::Vector3f(center.normal_x, center.normal_y, center.normal_z)) > cutGrazing) &&
					(laser.beamFalloff > cutFalloff))
					scanDataSet.push_back(ScanData(laser, px, std::sqrt(distance[idx])));
			}
		}

		return static_cast<int>(scanDataSet.size()) >= minRequireNumData;
	}
}