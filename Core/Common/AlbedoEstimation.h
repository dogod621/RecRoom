#pragma once

#include <vector>

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>

#include "Scan.h"
#include "Point.h"

namespace RecRoom
{
	class AlbedoEstimation : public pcl::Feature<PointMED, PointMED>
	{
	public:
		typedef boost::shared_ptr<AlbedoEstimation> Ptr;
		typedef boost::shared_ptr<const AlbedoEstimation> ConstPtr;
		using pcl::Feature<PointMED, PointMED>::feature_name_;
		using pcl::Feature<PointMED, PointMED>::getClassName;
		using pcl::Feature<PointMED, PointMED>::indices_;
		using pcl::Feature<PointMED, PointMED>::input_;
		using pcl::Feature<PointMED, PointMED>::surface_;
		using pcl::Feature<PointMED, PointMED>::k_;
		using pcl::Feature<PointMED, PointMED>::search_radius_;
		using pcl::Feature<PointMED, PointMED>::search_parameter_;

		typedef typename pcl::Feature<PointMED, PointMED>::PointCloudOut PointCloudOut;
		typedef typename pcl::Feature<PointMED, PointMED>::PointCloudConstPtr PointCloudConstPtr;

	public:
		AlbedoEstimation(const std::vector<ScanMeta>& scanMeta, 
			const LinearSolver linearSolver = LinearSolver::EIGEN_SVD, const double distInterParm = 10.0, const double angleInterParm = 20.0, const double cutFalloff = 0.33, const double cutGrazing = 0.86602540378)
			: scanMeta(scanMeta), linearSolver(linearSolver), 
			distInterParm(distInterParm), angleInterParm(angleInterParm), cutFalloff(cutFalloff), cutGrazing(cutGrazing)
		{
			feature_name_ = "AlbedoEstimation";
		};

		virtual ~AlbedoEstimation() {}

		virtual inline void setInputCloud(const PointCloudConstPtr &cloud)
		{
			input_ = cloud;
		}

		inline bool CollectScannLaserInfo(const pcl::PointCloud<PointMED>& cloud, const std::size_t k, const std::vector<int>& indices, const std::vector<float>& distance, const PointMED& inPoint, std::vector<ScannLaser>& scannLaser);

		inline bool ComputePointAlbedo(const std::vector<ScannLaser>& scannLaser, const PointMED& inPoint, PointMED& outPoint);


	protected:
		LinearSolver linearSolver;
		double distInterParm;
		double angleInterParm;
		double cutFalloff;
		double cutGrazing;

		std::vector<ScanMeta> scanMeta;

		void computeFeature(PointCloudOut &output);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	class AlbedoEstimationOMP : public AlbedoEstimation
	{
	public:
		typedef boost::shared_ptr<AlbedoEstimationOMP> Ptr;
		typedef boost::shared_ptr<const AlbedoEstimationOMP> ConstPtr;
		using AlbedoEstimation::feature_name_;
		using AlbedoEstimation::getClassName;
		using AlbedoEstimation::indices_;
		using AlbedoEstimation::input_;
		using AlbedoEstimation::surface_;
		using AlbedoEstimation::k_;
		using AlbedoEstimation::search_radius_;
		using AlbedoEstimation::search_parameter_;
		
		typedef typename AlbedoEstimation::PointCloudOut PointCloudOut;

		AlbedoEstimationOMP(const std::vector<ScanMeta>& scanMeta, 
			const LinearSolver linearSolver = LinearSolver::EIGEN_SVD, const double distInterParm = 10.0, const double angleInterParm = 20.0, const double cutFalloff = 0.33, const double cutGrazing = 0.86602540378,
			unsigned int nr_threads = 0)
			: AlbedoEstimation(scanMeta, linearSolver, distInterParm, angleInterParm, cutFalloff, cutGrazing)
		{
			feature_name_ = "AlbedoEstimationOMP";

			SetNumberOfThreads(nr_threads);
		}

		void SetNumberOfThreads(unsigned int nr_threads = 0);

	protected:
		unsigned int threads_;

	private:
		void computeFeature(PointCloudOut &output);
	};
}