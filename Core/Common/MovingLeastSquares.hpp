#pragma once

#include "MovingLeastSquares.h"

namespace RecRoom
{
	template <class PointT>
	void MLSResult::computeMLSSurface(
		const pcl::PointCloud<PointT> &cloud, int index, const std::vector<int> &nn_indices,
		double search_radius, int polynomial_order, boost::function<double(const double)> weight_func)
	{
		// Compute the plane coefficients
		EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix;
		Eigen::Vector4d xyz_centroid;

		// Estimate the XYZ centroid
		pcl::compute3DCentroid(cloud, nn_indices, xyz_centroid);

		// Compute the 3x3 covariance matrix
		pcl::computeCovarianceMatrix(cloud, nn_indices, xyz_centroid, covariance_matrix);
		EIGEN_ALIGN16 Eigen::Vector3d::Scalar eigen_value;
		EIGEN_ALIGN16 Eigen::Vector3d eigen_vector;
		Eigen::Vector4d model_coefficients(0, 0, 0, 0);
		pcl::eigen33(covariance_matrix, eigen_value, eigen_vector);
		model_coefficients.head<3>().matrix() = eigen_vector;
		model_coefficients[3] = -1 * model_coefficients.dot(xyz_centroid);

		// Projected query point
		valid = true;
		query_point = cloud.points[index].getVector3fMap().template cast<double>();
		double distance = query_point.dot(model_coefficients.head<3>()) + model_coefficients[3];
		mean = query_point - distance * model_coefficients.head<3>();

		curvature = covariance_matrix.trace();
		// Compute the curvature surface change
		if (curvature != 0)
			curvature = std::abs(eigen_value / curvature);

		// Get a copy of the plane normal easy access
		plane_normal = model_coefficients.head<3>();

		// Local coordinate system (Darboux frame)
		v_axis = plane_normal.unitOrthogonal();
		u_axis = plane_normal.cross(v_axis);

		// Perform polynomial fit to update point and normal
		////////////////////////////////////////////////////
		num_neighbors = static_cast<int> (nn_indices.size());
		order = polynomial_order;
		if (order > 1)
		{
			int nr_coeff = (order + 1) * (order + 2) / 2;

			if (num_neighbors >= nr_coeff)
			{
				// Note: The max_sq_radius parameter is only used if weight_func was not defined
				double max_sq_radius = 1;
				if (weight_func == 0)
				{
					max_sq_radius = search_radius * search_radius;
					weight_func = boost::bind(&pcl::MLSResult::computeMLSWeight, this, _1, max_sq_radius);
				}

				// Allocate matrices and vectors to hold the data used for the polynomial fit
				Eigen::VectorXd weight_vec(num_neighbors);
				Eigen::MatrixXd P(nr_coeff, num_neighbors);
				Eigen::VectorXd f_vec(num_neighbors);
				Eigen::MatrixXd P_weight; // size will be (nr_coeff_, nn_indices.size ());
				Eigen::MatrixXd P_weight_Pt(nr_coeff, nr_coeff);

				// Update neighborhood, since point was projected, and computing relative
				// positions. Note updating only distances for the weights for speed
				std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > de_meaned(num_neighbors);
				for (size_t ni = 0; ni < (size_t)num_neighbors; ++ni)
				{
					de_meaned[ni][0] = cloud.points[nn_indices[ni]].x - mean[0];
					de_meaned[ni][1] = cloud.points[nn_indices[ni]].y - mean[1];
					de_meaned[ni][2] = cloud.points[nn_indices[ni]].z - mean[2];
					weight_vec(ni) = weight_func(de_meaned[ni].dot(de_meaned[ni]));
				}

				// Go through neighbors, transform them in the local coordinate system,
				// save height and the evaluation of the polynome's terms
				double u_coord, v_coord, u_pow, v_pow;
				for (size_t ni = 0; ni < (size_t)num_neighbors; ++ni)
				{
					// Transforming coordinates
					u_coord = de_meaned[ni].dot(u_axis);
					v_coord = de_meaned[ni].dot(v_axis);
					f_vec(ni) = de_meaned[ni].dot(plane_normal);

					// Compute the polynomial's terms at the current point
					int j = 0;
					u_pow = 1;
					for (int ui = 0; ui <= order; ++ui)
					{
						v_pow = 1;
						for (int vi = 0; vi <= order - ui; ++vi)
						{
							P(j++, ni) = u_pow * v_pow;
							v_pow *= v_coord;
						}
						u_pow *= u_coord;
					}
				}

				// Computing coefficients
				P_weight = P * weight_vec.asDiagonal();
				P_weight_Pt = P_weight * P.transpose();
				c_vec = P_weight * f_vec;
				P_weight_Pt.llt().solveInPlace(c_vec);
			}
		}
	}

	template <class InPointNT>
	MLSVoxelGrid<InPointNT>::MLSVoxelGrid(PTR(Pc<InPointNT>)& cloud, PTR(PcIndex)& indices, float voxel_size)
		: voxel_grid_(), bounding_min_(), bounding_max_(), data_size_(), voxel_size_(voxel_size)
	{
		pcl::getMinMax3D(*cloud, *indices, bounding_min_, bounding_max_);

		Eigen::Vector4f bounding_box_size = bounding_max_ - bounding_min_;
		double max_size = (std::max) ((std::max)(bounding_box_size.x(), bounding_box_size.y()), bounding_box_size.z());
		// Put initial cloud in voxel grid
		data_size_ = static_cast<uint64_t> (1.5 * max_size / voxel_size_);
		for (unsigned int i = 0; i < indices->size(); ++i)
			if (pcl_isfinite(cloud->points[(*indices)[i]].x))
			{
				Eigen::Vector3i pos;
				getCellIndex(cloud->points[(*indices)[i]].getVector3fMap(), pos);

				uint64_t index_1d;
				getIndexIn1D(pos, index_1d);
				Leaf leaf;
				voxel_grid_[index_1d] = leaf;
			}
	}

	template <class InPointNT>
	void MLSVoxelGrid<InPointNT>::MLSVoxelGrid::dilate()
	{
		HashMap new_voxel_grid = voxel_grid_;
		for (typename MLSVoxelGrid::HashMap::iterator m_it = voxel_grid_.begin(); m_it != voxel_grid_.end(); ++m_it)
		{
			Eigen::Vector3i index;
			getIndexIn3D(m_it->first, index);

			// Now dilate all of its voxels
			for (int x = -1; x <= 1; ++x)
				for (int y = -1; y <= 1; ++y)
					for (int z = -1; z <= 1; ++z)
						if (x != 0 || y != 0 || z != 0)
						{
							Eigen::Vector3i new_index;
							new_index = index + Eigen::Vector3i(x, y, z);

							uint64_t index_1d;
							getIndexIn1D(new_index, index_1d);
							Leaf leaf;
							new_voxel_grid[index_1d] = leaf;
						}
		}
		voxel_grid_ = new_voxel_grid;
	}

	template <typename InPointNT, typename OutPointNT> 
	void MovingLeastSquares<InPointNT, OutPointNT>::process(Pc<OutPointNT>& output)
	{
		// Reset or initialize the collection of indices
		corresponding_input_indices_.reset(new pcl::PointIndices);

		// Copy the header
		output.header = input_->header;
		output.width = output.height = 0;
		output.points.clear();

		// Check if distinct_cloud_ was set
		if (upsample_method_ == MLSUpsamplingMethod::DISTINCT_CLOUD && !distinct_cloud_)
		{
			THROW_EXCEPTION("Upsample method was set to DISTINCT_CLOUD, but no distinct cloud was specified.");
			return;
		}

		switch (upsample_method_)
		{
			// Initialize random number generator if necessary
		case (MLSUpsamplingMethod::RANDOM_UNIFORM_DENSITY):
		{
			rng_alg_.seed(static_cast<unsigned> (std::time(0)));
			float tmp = static_cast<float> (search_radius_ / 2.0f);
			boost::uniform_real<float> uniform_distrib(-tmp, tmp);
			rng_uniform_distribution_.reset(new boost::variate_generator<boost::mt19937&, boost::uniform_real<float> >(rng_alg_, uniform_distrib));
			break;
		}
		case (MLSUpsamplingMethod::VOXEL_GRID_DILATION):
		case (MLSUpsamplingMethod::DISTINCT_CLOUD):
		{
			if (!cache_mls_results_)
				PCL_WARN("The cache mls results is forced when using upsampling method VOXEL_GRID_DILATION or DISTINCT_CLOUD");
			cache_mls_results_ = true;
			break;
		}
		default:
			break;
		}

		if (!initCompute())
			return;

		// Initialize the spatial locator
		if (!tree_)
		{
			KdTreePtr tree;
			if (input_->isOrganized())
				tree.reset(new pcl::search::OrganizedNeighbor<InPointNT>());
			else
				tree.reset(new pcl::search::KdTree<InPointNT>(false));
			setSearchMethod(tree);
		}

		// Send the surface dataset to the spatial locator
		tree_->setInputCloud(input_);

		if (cache_mls_results_)
		{
			mls_results_.resize(input_->size());
		}
		else
		{
			mls_results_.resize(1); // Need to have a reference to a single dummy result.
		}

		// Perform the actual surface reconstruction
		performProcessing(output);

		// Set proper widths and heights for the clouds
		output.height = 1;
		output.width = static_cast<uint32_t> (output.size());

		deinitCompute();
	}

	template <typename InPointNT, typename OutPointNT>
	void MovingLeastSquares<InPointNT, OutPointNT>::computeMLSPointNormal(int index, const std::vector<int> &nn_indices, Pc<OutPointNT>& projected_points, pcl::PointIndices &corresponding_input_indices, MLSResult &mls_result) const
	{
		mls_result.computeMLSSurface<InPointNT>(*input_, index, nn_indices, search_radius_, order_);

		switch (upsample_method_)
		{
		case (MLSUpsamplingMethod::MLSUpsamplingMethod_NONE):
		{
			MLSResult::MLSProjectionResults proj = mls_result.projectQueryPoint(projection_method_, nr_coeff_);
			addProjectedPointNormal(index, proj.point, proj.normal, mls_result.curvature, projected_points, corresponding_input_indices);
			break;
		}

		case (MLSUpsamplingMethod::SAMPLE_LOCAL_PLANE):
		{
			// Uniformly sample a circle around the query point using the radius and step parameters
			for (float u_disp = -static_cast<float> (upsampling_radius_); u_disp <= upsampling_radius_; u_disp += static_cast<float> (upsampling_step_))
				for (float v_disp = -static_cast<float> (upsampling_radius_); v_disp <= upsampling_radius_; v_disp += static_cast<float> (upsampling_step_))
					if (u_disp * u_disp + v_disp * v_disp < upsampling_radius_ * upsampling_radius_)
					{
						MLSResult::MLSProjectionResults proj = mls_result.projectPointSimpleToPolynomialSurface(u_disp, v_disp);
						addProjectedPointNormal(index, proj.point, proj.normal, mls_result.curvature, projected_points, corresponding_input_indices);
					}
			break;
		}

		case (MLSUpsamplingMethod::RANDOM_UNIFORM_DENSITY):
		{
			// Compute the local point density and add more samples if necessary
			int num_points_to_add = static_cast<int> (floor(desired_num_points_in_radius_ / 2.0 / static_cast<double> (nn_indices.size())));

			// Just add the query point, because the density is good
			if (num_points_to_add <= 0)
			{
				// Just add the current point
				MLSResult::MLSProjectionResults proj = mls_result.projectQueryPoint(projection_method_, nr_coeff_);
				addProjectedPointNormal(index, proj.point, proj.normal, mls_result.curvature, projected_points, corresponding_input_indices);
			}
			else
			{
				// Sample the local plane
				for (int num_added = 0; num_added < num_points_to_add;)
				{
					double u = (*rng_uniform_distribution_) ();
					double v = (*rng_uniform_distribution_) ();

					// Check if inside circle; if not, try another coin flip
					if (u * u + v * v > search_radius_ * search_radius_ / 4)
						continue;

					MLSResult::MLSProjectionResults proj;
					if (order_ > 1 && mls_result.num_neighbors >= 5 * nr_coeff_)
						proj = mls_result.projectPointSimpleToPolynomialSurface(u, v);
					else
						proj = mls_result.projectPointToMLSPlane(u, v);

					addProjectedPointNormal(index, proj.point, proj.normal, mls_result.curvature, projected_points, corresponding_input_indices);

					num_added++;
				}
			}
			break;
		}

		default:
			break;
		}
	}

	template <typename InPointNT, typename OutPointNT>
	void MovingLeastSquares<InPointNT, OutPointNT>::performProcessing(Pc<OutPointNT>& output)
	{
#ifdef _OPENMP
		// Create temporaries for each thread in order to avoid synchronization
		typename Pc<OutPointNT>::CloudVectorType projected_points(threads_);
		std::vector<pcl::PointIndices> corresponding_input_indices(threads_);
#endif

		// For all points
#ifdef _OPENMP
#pragma omp parallel for schedule (dynamic,1000) num_threads (threads_)
#endif
		for (int cp = 0; cp < static_cast<int> (indices_->size()); ++cp)
		{
			// Allocate enough space to hold the results of nearest neighbor searches
			// \note resize is irrelevant for a radiusSearch ().
			std::vector<int> nn_indices;
			std::vector<float> nn_sqr_dists;

			// Get the initial estimates of point positions and their neighborhoods
			if (searchForNeighbors((*indices_)[cp], nn_indices, nn_sqr_dists))
			{
				// Check the number of nearest neighbors for normal estimation (and later for polynomial fit as well)
				if (nn_indices.size() >= 3)
				{
					// This thread's ID (range 0 to threads-1)
#ifdef _OPENMP
					const int tn = omp_get_thread_num();
					// Size of projected points before computeMLSPointNormal () adds points
					size_t pp_size = projected_points[tn].size();
#else
					PointCloudOut projected_points;
#endif

					// Get a plane approximating the local surface's tangent and project point onto it
					const int index = (*indices_)[cp];

					size_t mls_result_index = 0;
					if (cache_mls_results_)
						mls_result_index = index; // otherwise we give it a dummy location.

#ifdef _OPENMP
					computeMLSPointNormal(index, nn_indices, projected_points[tn], corresponding_input_indices[tn], mls_results_[mls_result_index]);

					// Copy all information from the input cloud to the output points (not doing any interpolation)
					for (size_t pp = pp_size; pp < projected_points[tn].size(); ++pp)
						copyMissingFields(input_->points[(*indices_)[cp]], projected_points[tn][pp]);
#else
					computeMLSPointNormal(index, nn_indices, projected_points, *corresponding_input_indices_, mls_results_[mls_result_index]);

					// Append projected points to output
					output.insert(output.end(), projected_points.begin(), projected_points.end());
#endif
				}
			}
		}

#ifdef _OPENMP
		// Combine all threads' results into the output vectors
		for (unsigned int tn = 0; tn < threads_; ++tn)
		{
			output.insert(output.end(), projected_points[tn].begin(), projected_points[tn].end());
			corresponding_input_indices_->indices.insert(corresponding_input_indices_->indices.end(),
				corresponding_input_indices[tn].indices.begin(), corresponding_input_indices[tn].indices.end());
		}
#endif

		// Perform the distinct-cloud or voxel-grid upsampling
		performUpsampling(output);
	}

	template <typename InPointNT, typename OutPointNT>
	void MovingLeastSquares<InPointNT, OutPointNT>::performUpsampling(Pc<OutPointNT>& output)
	{
		if (upsample_method_ == MLSUpsamplingMethod::DISTINCT_CLOUD)
		{
			corresponding_input_indices_.reset(new PointIndices);
			for (size_t dp_i = 0; dp_i < distinct_cloud_->size(); ++dp_i) // dp_i = distinct_point_i
			{
				// Distinct cloud may have nan points, skip them
				if (!pcl_isfinite(distinct_cloud_->points[dp_i].x))
					continue;

				// Get 3D position of point
				//Eigen::Vector3f pos = distinct_cloud_->points[dp_i].getVector3fMap ();
				std::vector<int> nn_indices;
				std::vector<float> nn_dists;
				tree_->nearestKSearch(distinct_cloud_->points[dp_i], 1, nn_indices, nn_dists);
				int input_index = nn_indices.front();

				// If the closest point did not have a valid MLS fitting result
				// OR if it is too far away from the sampled point
				if (mls_results_[input_index].valid == false)
					continue;

				Eigen::Vector3d add_point = distinct_cloud_->points[dp_i].getVector3fMap().template cast<double>();
				MLSResult::MLSProjectionResults proj = mls_results_[input_index].projectPoint(add_point, projection_method_, 5 * nr_coeff_);
				addProjectedPointNormal(input_index, proj.point, proj.normal, mls_results_[input_index].curvature, output, *normals_, *corresponding_input_indices_);
			}
		}

		// For the voxel grid upsampling method, generate the voxel grid and dilate it
		// Then, project the newly obtained points to the MLS surface
		if (upsample_method_ == MLSUpsamplingMethod::VOXEL_GRID_DILATION)
		{
			corresponding_input_indices_.reset(new PointIndices);

			MLSVoxelGrid voxel_grid(input_, indices_, voxel_size_);
			for (int iteration = 0; iteration < dilation_iteration_num_; ++iteration)
				voxel_grid.dilate();

			for (typename MLSVoxelGrid::HashMap::iterator m_it = voxel_grid.voxel_grid_.begin(); m_it != voxel_grid.voxel_grid_.end(); ++m_it)
			{
				// Get 3D position of point
				Eigen::Vector3f pos;
				voxel_grid.getPosition(m_it->first, pos);

				InPointNT p;
				p.x = pos[0];
				p.y = pos[1];
				p.z = pos[2];

				std::vector<int> nn_indices;
				std::vector<float> nn_dists;
				tree_->nearestKSearch(p, 1, nn_indices, nn_dists);
				int input_index = nn_indices.front();

				// If the closest point did not have a valid MLS fitting result
				// OR if it is too far away from the sampled point
				if (mls_results_[input_index].valid == false)
					continue;

				Eigen::Vector3d add_point = p.getVector3fMap().template cast<double>();
				MLSResult::MLSProjectionResults proj = mls_results_[input_index].projectPoint(add_point, projection_method_, 5 * nr_coeff_);
				addProjectedPointNormal(input_index, proj.point, proj.normal, mls_results_[input_index].curvature, output, *normals_, *corresponding_input_indices_);
			}
		}
	}
}
