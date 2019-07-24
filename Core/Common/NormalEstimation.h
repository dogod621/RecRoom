#pragma once

#include <vector>

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>

#include "Common.h"
#include "Scanner/ScannerPc.h"

namespace RecRoom
{
	template<class InPointN, class OutPointN>
	class NormalEstimation : public pcl::Feature<InPointN, OutPointN>
	{
	public:
		typedef boost::shared_ptr<NormalEstimation<InPointN, OutPointN>> Ptr;
		typedef boost::shared_ptr<const NormalEstimation<InPointN, OutPointN>> ConstPtr;
		using pcl::Feature<InPointN, OutPointN>::feature_name_;
		using pcl::Feature<InPointN, OutPointN>::getClassName;
		using pcl::Feature<InPointN, OutPointN>::indices_;
		using pcl::Feature<InPointN, OutPointN>::input_;
		using pcl::Feature<InPointN, OutPointN>::surface_;
		using pcl::Feature<InPointN, OutPointN>::k_;
		using pcl::Feature<InPointN, OutPointN>::search_radius_;
		using pcl::Feature<InPointN, OutPointN>::search_parameter_;

		typedef typename pcl::Feature<InPointN, OutPointN>::PointCloudOut PointCloudOut;
		typedef typename pcl::Feature<InPointN, OutPointN>::PointCloudConstPtr PointCloudConstPtr;

	public:
		NormalEstimation(const CONST_PTR(ScannerPc)& scanner, unsigned int nr_threads = 0)
			: scanner(scanner), vpx(0), vpy(0), vpz(0),  useSensorOrigin(true), xyzCentroid(), covarianceMatrix()
		{
			if (!scanner)
				THROW_EXCEPTION("scanner is not set");

			feature_name_ = "NormalEstimation";
			SetNumberOfThreads(nr_threads);
		};

		virtual inline void setInputCloud(const PointCloudConstPtr &cloud)
		{
			input_ = cloud;
			if (useSensorOrigin)
			{
				vpx = input_->sensor_origin_.coeff(0);
				vpy = input_->sensor_origin_.coeff(1);
				vpz = input_->sensor_origin_.coeff(2);
			}
		}

		inline bool ComputePointNormal(const pcl::PointCloud<InPointN>& cloud, const std::vector<int>& indices,
			Eigen::Vector4f& planeParameters, float& curvature)
		{
			if (indices.size() < 3 ||
				pcl::computeMeanAndCovarianceMatrix(cloud, indices, covarianceMatrix, xyzCentroid) == 0)
			{
				planeParameters.setConstant(std::numeric_limits<float>::quiet_NaN());
				curvature = std::numeric_limits<float>::quiet_NaN();
				return false;
			}

			// Get the plane normal and surface curvature
			pcl::solvePlaneParameters(covarianceMatrix, xyzCentroid, planeParameters, curvature);
			return true;
		}

		bool ComputePointNormal(const pcl::PointCloud<InPointN>& cloud, const std::vector<int>& indices,
			float& nx, float& ny, float& nz, float& curvature)
		{
			if (indices.size() < 3 ||
				pcl::computeMeanAndCovarianceMatrix(cloud, indices, covarianceMatrix, xyzCentroid) == 0)
			{
				nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN();
				return false;
			}

			// Get the plane normal and surface curvature
			pcl::solvePlaneParameters(covarianceMatrix, nx, ny, nz, curvature);
			return true;
		}

		void SetNumberOfThreads(unsigned int nr_threads = 0)
		{
			if (nr_threads == 0)
#ifdef _OPENMP
				threads_ = omp_get_num_procs();
#else
				threads_ = 1;
#endif
			else
				threads_ = nr_threads;
		}
		
	protected:
		// brief Values describing the viewpoint ("pinhole" camera model assumed). 
		// For per point viewpoints, inherit from NormalEstimation and provide your own computeFeature (). 
		// By default, the viewpoint is set to 0,0,0.
		float vpx, vpy, vpz;

		// brief Placeholder for the 3x3 covariance matrix at each surface patch.
		EIGEN_ALIGN16 Eigen::Matrix3f covarianceMatrix;

		// brief 16-bytes aligned placeholder for the XYZ centroid of a surface patch.
		Eigen::Vector4f xyzCentroid;

		// whether the sensor origin of the input cloud or a user given viewpoint should be used.
		bool useSensorOrigin;

		CONST_PTR(ScannerPc) scanner;

		void computeFeature(PointCloudOut &output);

		unsigned int threads_;
		
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "NormalEstimation.hpp"
