#pragma once

#include "VoxelGrid.h"

#include "MovingLeastSquares.h"

namespace RecRoom
{
	template <class PointT>
	void MLSResult::computeMLSSurface(
		const pcl::PointCloud<PointT> &cloud, int index, const std::vector<int> &nnIndices,
		double searchRadius, int polynomialOrder, boost::function<double(const double)> WeightFunc)
	{
		// Compute the plane coefficients
		EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix;
		Eigen::Vector4d xyz_centroid;

		// Estimate the XYZ centroid
		pcl::compute3DCentroid(cloud, nnIndices, xyz_centroid);

		// Compute the 3x3 covariance matrix
		pcl::computeCovarianceMatrix(cloud, nnIndices, xyz_centroid, covariance_matrix);
		EIGEN_ALIGN16 Eigen::Vector3d::Scalar eigen_value;
		EIGEN_ALIGN16 Eigen::Vector3d eigen_vector;
		Eigen::Vector4d model_coefficients(0, 0, 0, 0);
		pcl::eigen33(covariance_matrix, eigen_value, eigen_vector);
		model_coefficients.head<3>().matrix() = eigen_vector;
		model_coefficients[3] = -1 * model_coefficients.dot(xyz_centroid);

		// Projected query point
		valid = true;
		queryPoint = cloud.points[index].getVector3fMap().template cast<double>();
		double distance = queryPoint.dot(model_coefficients.head<3>()) + model_coefficients[3];
		mean = queryPoint - distance * model_coefficients.head<3>();

		curvature = covariance_matrix.trace();
		// Compute the curvature surface change
		if (curvature != 0)
			curvature = std::abs(eigen_value / curvature);

		// Get a copy of the plane normal easy access
		planeNormal = model_coefficients.head<3>();

		// Local coordinate system (Darboux frame)
		vAxis = planeNormal.unitOrthogonal();
		uAxis = planeNormal.cross(vAxis);

		// Perform polynomial fit to update point and normal
		////////////////////////////////////////////////////
		numNeighbors = static_cast<int> (nnIndices.size());
		order = polynomialOrder;
		if (order > 1)
		{
			int nr_coeff = (order + 1) * (order + 2) / 2;

			if (numNeighbors >= nr_coeff)
			{
				// Note: The max_sq_radius parameter is only used if WeightFunc was not defined
				double max_sq_radius = 1;
				if (WeightFunc == 0)
				{
					max_sq_radius = searchRadius * searchRadius;
					WeightFunc = boost::bind(&pcl::MLSResult::computeMLSWeight, this, _1, max_sq_radius);
				}

				// Allocate matrices and vectors to hold the data used for the polynomial fit
				Eigen::VectorXd weight_vec(numNeighbors);
				Eigen::MatrixXd P(nr_coeff, numNeighbors);
				Eigen::VectorXd f_vec(numNeighbors);
				Eigen::MatrixXd P_weight; // size will be (nr_coeff_, nnIndices.size ());
				Eigen::MatrixXd P_weight_Pt(nr_coeff, nr_coeff);

				// Update neighborhood, since point was projected, and computing relative
				// positions. Note updating only distances for the weights for speed
				std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > de_meaned(numNeighbors);
				for (size_t ni = 0; ni < (size_t)numNeighbors; ++ni)
				{
					de_meaned[ni][0] = cloud.points[nnIndices[ni]].x - mean[0];
					de_meaned[ni][1] = cloud.points[nnIndices[ni]].y - mean[1];
					de_meaned[ni][2] = cloud.points[nnIndices[ni]].z - mean[2];
					weight_vec(ni) = WeightFunc(de_meaned[ni].dot(de_meaned[ni]));
				}

				// Go through neighbors, transform them in the local coordinate system,
				// save height and the evaluation of the polynome's terms
				double u_coord, v_coord, u_pow, v_pow;
				for (size_t ni = 0; ni < (size_t)numNeighbors; ++ni)
				{
					// Transforming coordinates
					u_coord = de_meaned[ni].dot(uAxis);
					v_coord = de_meaned[ni].dot(vAxis);
					f_vec(ni) = de_meaned[ni].dot(planeNormal);

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
				cAxis = P_weight * f_vec;
				P_weight_Pt.llt().solveInPlace(cAxis);
			}
		}
	}

	template <typename InPointN, typename OutPointN> 
	void MovingLeastSquares<InPointN, OutPointN>::process(Pc<OutPointN>& output)
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
				tree.reset(new pcl::search::OrganizedNeighbor<InPointN>());
			else
				tree.reset(new pcl::search::KdTree<InPointN>(false));
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

	template <typename InPointN, typename OutPointN>
	void MovingLeastSquares<InPointN, OutPointN>::computeMLSPointNormal(int index, const std::vector<int> &nnIndices, Pc<OutPointN>& projected_points, pcl::PointIndices &corresponding_input_indices, MLSResult &mls_result) const
	{
		mls_result.computeMLSSurface<InPointN>(*input_, index, nnIndices, search_radius_, order_);

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
			int num_points_to_add = static_cast<int> (floor(desired_num_points_in_radius_ / 2.0 / static_cast<double> (nnIndices.size())));

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
					if (order_ > 1 && mls_result.numNeighbors >= 5 * nr_coeff_)
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

	template <typename InPointN, typename OutPointN>
	void MovingLeastSquares<InPointN, OutPointN>::performProcessing(Pc<OutPointN>& output)
	{
#ifdef _OPENMP
		// Create temporaries for each thread in order to avoid synchronization
		typename Pc<OutPointN>::CloudVectorType projected_points(threads_);
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
			std::vector<int> nnIndices;
			std::vector<float> nn_sqr_dists;

			// Get the initial estimates of point positions and their neighborhoods
			if (searchForNeighbors((*indices_)[cp], nnIndices, nn_sqr_dists))
			{
				// Check the number of nearest neighbors for normal estimation (and later for polynomial fit as well)
				if (nnIndices.size() >= 3)
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
					computeMLSPointNormal(index, nnIndices, projected_points[tn], corresponding_input_indices[tn], mls_results_[mls_result_index]);

					// Copy all information from the input cloud to the output points (not doing any interpolation)
					for (size_t pp = pp_size; pp < projected_points[tn].size(); ++pp)
						copyMissingFields(input_->points[(*indices_)[cp]], projected_points[tn][pp]);
#else
					computeMLSPointNormal(index, nnIndices, projected_points, *corresponding_input_indices_, mls_results_[mls_result_index]);

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

	template <typename InPointN, typename OutPointN>
	void MovingLeastSquares<InPointN, OutPointN>::performUpsampling(Pc<OutPointN>& output)
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
				std::vector<int> nnIndices;
				std::vector<float> nn_dists;
				tree_->nearestKSearch(distinct_cloud_->points[dp_i], 1, nnIndices, nn_dists);
				int input_index = nnIndices.front();

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

			Eigen::Vector4f minAABB, maxAABB;
			pcl::getMinMax3D(*input_, *indices_, minAABB, maxAABB);

			BinaryVoxelGrid<InPointN> voxelGrid(
				voxel_size_,
				Eigen::Vector3d(minAABB.x(), minAABB.y(), minAABB.z()),
				Eigen::Vector3d(maxAABB.x(), maxAABB.y(), maxAABB.z()));

			voxelGrid.AddPointCloud(input_, indices_)
			voxelGrid.Dilation(1, dilation_iteration_num_);
			PTR(Pc<PointType>) pcDilat = voxelGrid.GetPointCloud();

			for (Pc<InPointN>::const_iterator it = pcDilat->begin(); it != pcDilat->end(); ++it)
			{
				// Get 3D position of point
				std::vector<int> nnIndices;
				std::vector<float> nn_dists;
				tree_->nearestKSearch(*it, 1, nnIndices, nn_dists);
				int input_index = nnIndices.front();

				// If the closest point did not have a valid MLS fitting result
				// OR if it is too far away from the sampled point
				if (mls_results_[input_index].valid == false)
					continue;

				Eigen::Vector3d add_point = it->getVector3fMap().template cast<double>();
				MLSResult::MLSProjectionResults proj = mls_results_[input_index].projectPoint(add_point, projection_method_, 5 * nr_coeff_);
				addProjectedPointNormal(input_index, proj.point, proj.normal, mls_results_[input_index].curvature, output, *normals_, *corresponding_input_indices_);
			}
		}
	}
}
