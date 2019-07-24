#pragma once

#include "NormalEstimation.h"

namespace RecRoom
{
	template<class InPointN, class OutPointN>
	void NormalEstimation<InPointN, OutPointN>::computeFeature(PointCloudOut &output)
	{
		if (!(search_radius_ > 0.0))
			THROW_EXCEPTION("search_radius_ is not set");

		std::vector<int> nn_indices(k_);
		std::vector<float> nn_dists(k_);

		output.is_dense = true;
		if (input_->is_dense)
		{
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				const PointMED& inPoint = (*input_)[(*indices_)[idx]];
				PointMED& outPoint = output.points[idx];

				if (searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) > 0)
				{
					if(!ComputePointNormal(*surface_, nn_indices, outPoint.normal[0], outPoint.normal[1], outPoint.normal[2], outPoint.curvature))
					{
						PRINT_WARNING("ComputePointNormal failed");
						outPoint.normal[0] = outPoint.normal[1] = outPoint.normal[2] = outPoint.curvature = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PRINT_WARNING("searchForNeighbors failed");
					outPoint.normal[0] = outPoint.normal[1] = outPoint.normal[2] = outPoint.curvature = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
				}
			}
		}
		else
		{
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				const PointMED& inPoint = (*input_)[(*indices_)[idx]];
				PointMED& outPoint = output.points[idx];
				if (pcl::isFinite(inPoint))
				{
					if (searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) > 0)
					{
						if (!ComputePointNormal(*surface_, nn_indices, outPoint.normal[0], outPoint.normal[1], outPoint.normal[2], outPoint.curvature))
						{
							PRINT_WARNING("ComputePointNormal failed");
							outPoint.normal[0] = outPoint.normal[1] = outPoint.normal[2] = outPoint.curvature = std::numeric_limits<float>::quiet_NaN();
							output.is_dense = false;
						}
					}
					else
					{
						PRINT_WARNING("searchForNeighbors failed");
						outPoint.normal[0] = outPoint.normal[1] = outPoint.normal[2] = outPoint.curvature = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PRINT_WARNING("Input point contain non finite value");
					outPoint.normal[0] = outPoint.normal[1] = outPoint.normal[2] = outPoint.curvature = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
				}
			}
		}
	}
}