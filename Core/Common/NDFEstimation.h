#pragma once

#include <vector>

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>

#include "Common.h"
#include "Scanner/ScannerPc.h"

namespace RecRoom
{
	template<class InPointIN, class OutPointINS>
	class NDFEstimation : public pcl::Feature<InPointIN, OutPointINS>
	{
	public:
		typedef boost::shared_ptr<NDFEstimation<InPointIN, OutPointINS>> Ptr;
		typedef boost::shared_ptr<const NDFEstimation<InPointIN, OutPointINS>> ConstPtr;
		using pcl::Feature<InPointIN, OutPointINS>::feature_name_;
		using pcl::Feature<InPointIN, OutPointINS>::getClassName;
		using pcl::Feature<InPointIN, OutPointINS>::indices_;
		using pcl::Feature<InPointIN, OutPointINS>::input_;
		using pcl::Feature<InPointIN, OutPointINS>::surface_;
		using pcl::Feature<InPointIN, OutPointINS>::k_;
		using pcl::Feature<InPointIN, OutPointINS>::search_radius_;
		using pcl::Feature<InPointIN, OutPointINS>::search_parameter_;

		typedef typename pcl::Feature<InPointIN, OutPointINS>::PointCloudOut PointCloudOut;
		typedef typename pcl::Feature<InPointIN, OutPointINS>::PointCloudConstPtr PointCloudConstPtr;

	public:
		NDFEstimation(const CONST_PTR(ScannerPc)& scanner,
			const LinearSolver linearSolver = LinearSolver::EIGEN_SVD,
			const double distInterParm = 10.0, const double angleInterParm = 20.0, const double cutFalloff = 0.33,
			const double cutGrazing = 0.86602540378,
			unsigned int nr_threads = 0)
			: scanner(scanner), linearSolver(linearSolver),
			distInterParm(distInterParm), angleInterParm(angleInterParm), cutFalloff(cutFalloff),
			cutGrazing(cutGrazing)
		{
			if (!scanner)
				THROW_EXCEPTION("scanner is not set");

			feature_name_ = "NDFEstimation";
			SetNumberOfThreads(nr_threads);
		};

		virtual inline void setInputCloud(const PointCloudConstPtr &cloud)
		{
			input_ = cloud;
		}

		inline bool CollectScanLaserInfo(const Pc<InPointIN>& cloud, const std::size_t k, const std::vector<int>& indices, const std::vector<float>& distance, const InPointIN& inPoint, std::vector<ScanLaser>& scanLaserSet);

		inline bool ComputePointAlbedo(const std::vector<ScanLaser>& scanLaserSet, const InPointIN& inPoint, OutPointINS& outPoint);

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
		LinearSolver linearSolver;
		double distInterParm;
		double angleInterParm;
		double cutFalloff;
		double cutGrazing;
		CONST_PTR(ScannerPc) scanner;

		void computeFeature(PointCloudOut &output);

		unsigned int threads_;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "NDFEstimation.hpp"
