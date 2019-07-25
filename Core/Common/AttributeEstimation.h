#pragma once

#include <vector>

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>

#include "Common.h"
#include "Scanner/ScannerPc.h"

namespace RecRoom
{
	struct ScanData
	{
		ScanLaser laser;
		int index;
		double distance2Center;

		ScanData(const ScanLaser& laser = ScanLaser(), int index = -1, double distance2Center = 0.0): laser(laser), index(index), distance2Center(distance2Center) {}
	};

	template<class InPointType, class OutPointType>
	class SurfaceEstimation : public pcl::Feature<InPointType, OutPointType>
	{
	public:
		using pcl::Feature<InPointType, OutPointType>::feature_name_;
		using pcl::Feature<InPointType, OutPointType>::getClassName;
		using pcl::Feature<InPointType, OutPointType>::indices_;
		using pcl::Feature<InPointType, OutPointType>::input_;
		using pcl::Feature<InPointType, OutPointType>::surface_;
		using pcl::Feature<InPointType, OutPointType>::k_;
		using pcl::Feature<InPointType, OutPointType>::search_radius_;
		using pcl::Feature<InPointType, OutPointType>::search_parameter_;

		typedef typename pcl::Feature<InPointType, OutPointType>::PointCloudOut PointCloudOut;
		typedef typename pcl::Feature<InPointType, OutPointType>::PointCloudConstPtr PointCloudConstPtr;

	public:
		SurfaceEstimation(const CONST_PTR(ScannerPc)& scanner,
			const double cutFalloff = 0.33, // cut attinuation less than 1/3
			const std::size_t minRequireNumData = 1,
			unsigned int numThreads = 0)
			: scanner(scanner), cutFalloff(cutFalloff), minRequireNumData(minRequireNumData)
		{
			feature_name_ = "SurfaceEstimation";

			if (!scanner)
				THROW_EXCEPTION("scanner is not set");

			if(minRequireNumData == 0)
				THROW_EXCEPTION("minRequireNumData == 0");

			SetNumberOfThreads(numThreads);
		};

		virtual inline void setInputCloud(const PointCloudConstPtr& cloud)
		{
			input_ = cloud;
		}

		void SetNumberOfThreads(unsigned int numThreads = 0)
		{
			if (numThreads == 0)
#ifdef _OPENMP
				threads_ = omp_get_num_procs();
#else
				threads_ = 1;
#endif
			else
				threads_ = numThreads;
		}

	protected:
		inline virtual bool CollectScanData(
			const Pc<InPointType>& cloud, const std::size_t k, const std::vector<int>& indices, const std::vector<float>& distance,
			const InPointType& center, std::vector<ScanData>& scanDataSet) const;

		inline virtual bool ComputeAttribute(
			const Pc<InPointType>& cloud, 
			const InPointType& center, const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const = 0;

		inline virtual void SetAttributeNAN(OutPointType& p) const = 0;

		void computeFeature(PointCloudOut& output);

	protected:
		CONST_PTR(ScannerPc) scanner;

		double cutFalloff;

		std::size_t minRequireNumData;

		unsigned int threads_;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template<class InPointType, class OutPointType>
	class AttributeEstimation : public SurfaceEstimation<InPointType, OutPointType>
	{
	public:
		AttributeEstimation(const CONST_PTR(ScannerPc)& scanner,
			const double cutFalloff = 0.33, // cut attinuation less than 1/3
			const double cutGrazing = 1.3, // cut icident agngle larger than 75 degrees
			const std::size_t minRequireNumData = 1,
			unsigned int numThreads = 0)
			: SurfaceEstimation<InPointType, OutPointType>(scanner, cutFalloff, minRequireNumData, numThreads), cutGrazing(cutGrazing)
		{
			feature_name_ = "AttributeEstimation";
		};

	protected:
		inline virtual bool CollectScanData(
			const Pc<InPointType>& cloud, const std::size_t k, const std::vector<int>& indices, const std::vector<float>& distance, 
			const InPointType& center, std::vector<ScanData>& scanDataSet) const;

	protected:
		double cutGrazing;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "AttributeEstimation.hpp"
