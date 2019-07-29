#pragma once

#include "VoxelGrid.h"
#include "Random.h"

#include "MovingLeastSquares.h"

namespace RecRoom
{
	template <class PointT>
	void MLSResult::computeMLSSurface(
		const pcl::PointCloud<PointT> &cloud, int index, const PcIndex& nnIndices,
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
				Eigen::MatrixXd P_weight; // size will be (numCoeff, nnIndices.size ());
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
		correspondingInputIndices.reset(new PcIndex);

		// Copy the header
		output.header = input_->header;
		output.width = output.height = 0;
		output.points.clear();

		// Check if distinctCloud was set
		if ((upsampleMethod == MLSUpsamplingMethod::DISTINCT_CLOUD) && !distinctCloud)
		{
			THROW_EXCEPTION("Upsample method was set to DISTINCT_CLOUD, but no distinct cloud was specified.");
			return;
		}

		//
		switch (upsampleMethod)
		{
		case (MLSUpsamplingMethod::VOXEL_GRID_DILATION):
		case (MLSUpsamplingMethod::DISTINCT_CLOUD):
		{
			if (!cacheMLSResults)
				PRINT_WARNING("The cache mls results is forced when using upsampling method VOXEL_GRID_DILATION or DISTINCT_CLOUD, set to true.");
			cacheMLSResults = true;
			break;
		}
		default:
			break;
		}

		if (!initCompute())
			return;

		// Make sure the searchMethod searches the surface
		if (!searchMethod)
		{
			PRINT_WARNING("searchMethod is not set, creat.");
			if (input_->isOrganized())
				searchMethod.reset(new pcl::search::OrganizedNeighbor<InPointN>());
			else
				searchMethod.reset(new pcl::search::KdTree<InPointN>(false));
		}

		if (searchMethod->getInputCloud() != input_)
		{
			PRINT_WARNING("searchMethod is not search on input, rebuild it.");
			searchMethod->setInputCloud(surface_);
		}

		if (cacheMLSResults)
		{
			mlsResults.resize(input_->size());
		}

		// Perform the actual surface reconstruction
		performProcessing(output);

		// Set proper widths and heights for the clouds
		output.height = 1;
		output.width = static_cast<uint32_t> (output.size());

		deinitCompute();
	}

	template <typename InPointN, typename OutPointN>
	void MovingLeastSquares<InPointN, OutPointN>::computeMLSPointNormal(int index, const PcIndex& nnIndices, Pc<OutPointN>& projectedPoints, PcIndex& correspondingInputIndices_, MLSResult &mlsResult) const
	{
		mlsResult.computeMLSSurface<InPointN>(*input_, index, nnIndices, searchRadius, order);

		switch (upsampleMethod)
		{
		case (MLSUpsamplingMethod::MLSUpsamplingMethod_NONE):
		{
			MLSResult::MLSProjectionResults proj = mlsResult.projectQueryPoint(projectionMethod, numCoeff);
			addProjectedPointNormal(index, proj.point, proj.normal, mlsResult.curvature, projectedPoints, correspondingInputIndices_);
			break;
		}

		case (MLSUpsamplingMethod::SAMPLE_LOCAL_PLANE):
		{
			// Uniformly sample a circle around the query point using the radius and step parameters
			for (float u_disp = -static_cast<float> (upsamplingRadius); u_disp <= upsamplingRadius; u_disp += static_cast<float> (upsamplingStep))
				for (float v_disp = -static_cast<float> (upsamplingRadius); v_disp <= upsamplingRadius; v_disp += static_cast<float> (upsamplingStep))
					if (u_disp * u_disp + v_disp * v_disp < upsamplingRadius * upsamplingRadius)
					{
						MLSResult::MLSProjectionResults proj = mlsResult.projectPointSimpleToPolynomialSurface(u_disp, v_disp);
						addProjectedPointNormal(index, proj.point, proj.normal, mlsResult.curvature, projectedPoints, correspondingInputIndices_);
					}
			break;
		}

		case (MLSUpsamplingMethod::RANDOM_UNIFORM_DENSITY):
		{
			// Compute the local point density and add more samples if necessary
			int num_points_to_add = static_cast<int> (floor(pointDensity / 2.0 / static_cast<double> (nnIndices.size())));

			// Just add the query point, because the density is good
			if (num_points_to_add <= 0)
			{
				// Just add the current point
				MLSResult::MLSProjectionResults proj = mlsResult.projectQueryPoint(projectionMethod, numCoeff);
				addProjectedPointNormal(index, proj.point, proj.normal, mlsResult.curvature, projectedPoints, correspondingInputIndices_);
			}
			else
			{
				// Sample the local plane
				for (int num_added = 0; num_added < num_points_to_add;)
				{
					double u = (Random::Uniform<float>() - 0.5f) * searchRadius;
					double v = (Random::Uniform<float>() - 0.5f) * searchRadius;

					// Check if inside circle; if not, try another coin flip
					if (u * u + v * v > searchRadius * searchRadius * 0.25f)
						continue;

					MLSResult::MLSProjectionResults proj;
					if (order > 1 && mlsResult.numNeighbors >= 5 * numCoeff)
						proj = mlsResult.projectPointSimpleToPolynomialSurface(u, v);
					else
						proj = mlsResult.projectPointToMLSPlane(u, v);

					addProjectedPointNormal(index, proj.point, proj.normal, mlsResult.curvature, projectedPoints, correspondingInputIndices_);

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
		typename Pc<OutPointN>::CloudVectorType projectedPointSet(numThreads);
		std::vector<PcIndex> correspondingInputIndicesSet(numThreads);
#endif

		// For all points
#ifdef _OPENMP
#pragma omp parallel for schedule (dynamic,1000) num_threads (numThreads)
#endif
		for (int cp = 0; cp < static_cast<int> (indices_->size()); ++cp)
		{
			// Get a plane approximating the local surface's tangent and project point onto it
			const int index = (*indices_)[cp];
			InOutPointN& inP = (*input_)[index];

			// Allocate enough space to hold the results of nearest neighbor searches
			// \note resize is irrelevant for a radiusSearch ().
			PcIndex nnIndices;
			std::vector<float> nnSqrDists;

			// Get the initial estimates of point positions and their neighborhoods
			if (searchMethod->radiusSearch(inP, searchRadius, nnIndices, nnSqrDists))
			{
				// Filter back face
				PcIndex nnFrontIndices;
				nnFrontIndices.reserve(nnSqrDists.size());
				{
					for (PcIndex::const_iterator it = nnIndices.begin(); it != nnIndices.end(); ++it)
					{
						InOutPointN& neiP = (*indices_)[(*it)];
						if ((inP.normal_x * neiP.normal_x + inP.normal_y * neiP.normal_y + inP.normal_z * neiP.normal_z) > 0)
							nnFrontIndices.push_back(*it);
					}
				}

				// Check the number of nearest neighbors for normal estimation (and later for polynomial fit as well)
				if (nnFrontIndices.size() >= 3)
				{
					// This thread's ID (range 0 to threads-1)
#ifdef _OPENMP
					const int tn = omp_get_thread_num();
					// Size of projected points before computeMLSPointNormal () adds points
					size_t pp_size = projectedPointSet[tn].size();
#else
					PointCloudOut projectedPoints;
#endif

#ifdef _OPENMP
					if (cacheMLSResults)
						computeMLSPointNormal(index, nnFrontIndices, projectedPointSet[tn], correspondingInputIndicesSet[tn], mlsResults[index]);
					else
					{
						MLSResult temp;
						computeMLSPointNormal(index, nnFrontIndices, projectedPointSet[tn], correspondingInputIndicesSet[tn], temp);
					}

#else
					if (cacheMLSResults)
						computeMLSPointNormal(index, nnFrontIndices, projectedPoints, *correspondingInputIndices, mlsResults[index]);
					else
					{
						MLSResult temp;
						computeMLSPointNormal(index, nnFrontIndices, projectedPoints, *correspondingInputIndices, temp);
					}

					// Append projected points to output
					output.insert(output.end(), projectedPoints.begin(), projectedPoints.end());
#endif
				}
			}
		}

#ifdef _OPENMP
		// Combine all threads' results into the output vectors
		for (unsigned int tn = 0; tn < numThreads; ++tn)
		{
			output.insert(output.end(), projectedPoints[tn].begin(), projectedPoints[tn].end());
			correspondingInputIndices->insert(correspondingInputIndices.end(), correspondingInputIndicesSet[tn].begin(), correspondingInputIndicesSet[tn].end());
		}
#endif

		// Perform the distinct-cloud or voxel-grid upsampling
		performUpsampling(output);
	}

	template <typename InPointN, typename OutPointN>
	void MovingLeastSquares<InPointN, OutPointN>::performUpsampling(Pc<OutPointN>& output)
	{
		if (upsampleMethod == MLSUpsamplingMethod::DISTINCT_CLOUD)
		{
			correspondingInputIndices.reset(new PcIndex);
			for (size_t dp_i = 0; dp_i < distinctCloud->size(); ++dp_i) // dp_i = distinct_point_i
			{
				// Distinct cloud may have nan points, skip them
				if (!pcl_isfinite(distinctCloud->points[dp_i].x))
					continue;

				// Get 3D position of point
				//Eigen::Vector3f pos = distinctCloud->points[dp_i].getVector3fMap ();
				PcIndex nnIndices;
				std::vector<float> nn_dists;
				searchMethod->nearestKSearch(distinctCloud->points[dp_i], 1, nnIndices, nn_dists);
				int input_index = nnIndices.front();

				// If the closest point did not have a valid MLS fitting result
				// OR if it is too far away from the sampled point
				if (mlsResults[input_index].valid == false)
					continue;

				Eigen::Vector3d add_point = distinctCloud->points[dp_i].getVector3fMap().template cast<double>();
				MLSResult::MLSProjectionResults proj = mlsResults[input_index].projectPoint(add_point, projectionMethod, 5 * numCoeff);
				addProjectedPointNormal(input_index, proj.point, proj.normal, mlsResults[input_index].curvature, output, *correspondingInputIndices);
			}
		}

		// For the voxel grid upsampling method, generate the voxel grid and dilate it
		// Then, project the newly obtained points to the MLS surface
		if (upsampleMethod == MLSUpsamplingMethod::VOXEL_GRID_DILATION)
		{
			correspondingInputIndices.reset(new PcIndex);

			Eigen::Vector4f minAABB, maxAABB;
			pcl::getMinMax3D(*input_, *indices_, minAABB, maxAABB);

			BinaryVoxelGrid<InPointN> voxelGrid(
				dilationVoxelSize,
				Eigen::Vector3d(minAABB.x(), minAABB.y(), minAABB.z()),
				Eigen::Vector3d(maxAABB.x(), maxAABB.y(), maxAABB.z()));

			voxelGrid.AddPointCloud(input_, indices_)
			voxelGrid.Dilation(dilationKernalSize, dilationIterations);
			PTR(Pc<PointType>) pcDilat = voxelGrid.GetPointCloud();

			for (Pc<InPointN>::const_iterator it = pcDilat->begin(); it != pcDilat->end(); ++it)
			{
				// Get 3D position of point
				PcIndex nnIndices;
				std::vector<float> nn_dists;
				searchMethod->nearestKSearch(*it, 1, nnIndices, nn_dists);
				int input_index = nnIndices.front();

				// If the closest point did not have a valid MLS fitting result
				// OR if it is too far away from the sampled point
				if (mlsResults[input_index].valid == false)
					continue;

				Eigen::Vector3d add_point = it->getVector3fMap().template cast<double>();
				MLSResult::MLSProjectionResults proj = mlsResults[input_index].projectPoint(add_point, projectionMethod, 5 * numCoeff);
				addProjectedPointNormal(input_index, proj.point, proj.normal, mlsResults[input_index].curvature, output, *correspondingInputIndices);
			}
		}
	}
}
