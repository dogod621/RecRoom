#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/common/common.h>
#include <pcl/common/vector_average.h>
#include <pcl/Vertices.h>

#include "MarchingCubesRBF.h"

namespace RecRoom
{
	void MarchingCubesRBF::voxelizeData()
	{
		// Initialize data structures
		const unsigned int N = static_cast<unsigned int> (input_->size());
		Eigen::MatrixXd M(2 * N, 2 * N), d(2 * N, 1);

		for (unsigned int row_i = 0; row_i < 2 * N; ++row_i)
		{
			// boolean variable to determine whether we are in the off_surface domain for the rows
			bool row_off = (row_i >= N) ? 1 : 0;
			for (unsigned int col_i = 0; col_i < 2 * N; ++col_i)
			{
				// boolean variable to determine whether we are in the off_surface domain for the columns
				bool col_off = (col_i >= N) ? 1 : 0;
				M(row_i, col_i) = kernel(
					Eigen::Vector3f(input_->points[col_i%N].getVector3fMap()).cast<double>() + 
					Eigen::Vector3f(input_->points[col_i%N].getNormalVector3fMap()).cast<double>() * col_off * off_surface_epsilon_,

					Eigen::Vector3f(input_->points[row_i%N].getVector3fMap()).cast<double>() + 
					Eigen::Vector3f(input_->points[row_i%N].getNormalVector3fMap()).cast<double>() * row_off * off_surface_epsilon_);
			}

			d(row_i, 0) = row_off * off_surface_epsilon_;
		}

		// Solve for the weights
		Eigen::MatrixXd w(2 * N, 1);

		// Solve_linear_system (M, d, w);
		w = M.fullPivLu().solve(d);

		std::vector<double> weights(2 * N);
		std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > centers(2 * N);
		for (unsigned int i = 0; i < N; ++i)
		{
			centers[i] = Eigen::Vector3f(input_->points[i].getVector3fMap()).cast<double>();
			centers[i + N] = Eigen::Vector3f(input_->points[i].getVector3fMap()).cast<double>() + Eigen::Vector3f(input_->points[i].getNormalVector3fMap()).cast<double>() * off_surface_epsilon_;
			weights[i] = w(i, 0);
			weights[i + N] = w(i + N, 0);
		}

		for (int x = 0; x < res_x_; ++x)
			for (int y = 0; y < res_y_; ++y)
				for (int z = 0; z < res_z_; ++z)
				{
					const Eigen::Vector3f point_f = (size_voxel_ * Eigen::Array3f(x, y, z) + lower_boundary_).matrix();
					const Eigen::Vector3d point = point_f.cast<double>();

					double f = 0.0;
					std::vector<double>::const_iterator w_it(weights.begin());
					for (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >::const_iterator c_it = centers.begin();
						c_it != centers.end(); ++c_it, ++w_it)
						f += *w_it * kernel(*c_it, point);

					grid_[x * res_y_*res_z_ + y * res_z_ + z] = float(f);
				}
	}
}
