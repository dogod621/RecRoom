#pragma once

#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/common/common.h>
#include <pcl/common/vector_average.h>
#include <pcl/Vertices.h>

#include "MarchingCubesHoppe.h"

namespace RecRoom
{
	template<class PointN>
	void MarchingCubesHoppe<PointN>::voxelizeData()
	{
		const bool is_far_ignored = dist_ignore_ > 0.0f;

		for (int x = 0; x < res_x_; ++x)
		{
			const int y_start = x * res_y_ * res_z_;

			for (int y = 0; y < res_y_; ++y)
			{
				const int z_start = y_start + y * res_z_;

				for (int z = 0; z < res_z_; ++z)
				{
					std::vector<int> nn_indices(1, 0);
					std::vector<float> nn_sqr_dists(1, 0.0f);
					const Eigen::Vector3f point = (lower_boundary_ + size_voxel_ * Eigen::Array3f(x, y, z)).matrix();
					PointN p;

					p.getVector3fMap() = point;

					tree_->nearestKSearch(p, 1, nn_indices, nn_sqr_dists);

					if (!is_far_ignored || nn_sqr_dists[0] < dist_ignore_)
					{
						const Eigen::Vector3f normal = input_->points[nn_indices[0]].getNormalVector3fMap();

						if (!std::isnan(normal(0)) && normal.norm() > 0.5f)
							grid_[z_start + z] = normal.dot(
								point - input_->points[nn_indices[0]].getVector3fMap());
					}
				}
			}
		}
	}
}
