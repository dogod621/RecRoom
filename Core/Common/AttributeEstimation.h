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
		float distance2Center;

		ScanData(const ScanLaser& laser = ScanLaser(), int index = -1, float distance2Center = 0.0f): laser(laser), index(index), distance2Center(distance2Center) {}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template<class InPointType, class OutPointType>
	class SurfaceEstimation;

	template<class InPointType, class OutPointType>
	void EstimationTask(int id, SurfaceEstimation<InPointType, OutPointType>* self, Pc<OutPointType>* output);

	template<class InPointType, class OutPointType>
	class SurfaceEstimation : public pcl::Feature<InPointType, OutPointType>, public ThreadAble
	{
	public:
		friend void EstimationTask<InPointType, OutPointType> (int id, SurfaceEstimation<InPointType, OutPointType>* self, Pc<OutPointType>* output);

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
			const float distInterParm = 0.4f,
			const float cutFalloff = 0.33f, // cut attinuation less than 1/3
			const int minRequireNumData = 1)
			: ThreadAble(), scanner(scanner), distInterParm(distInterParm), cutFalloff(cutFalloff), minRequireNumData(minRequireNumData)
		{
			feature_name_ = "SurfaceEstimation";

			if (!scanner)
				THROW_EXCEPTION("scanner is not set");

			if(minRequireNumData < 1)
				THROW_EXCEPTION("minRequireNumData < 1");
		};

		virtual inline void setInputCloud(const PointCloudConstPtr& cloud)
		{
			input_ = cloud;
		}

	protected:
		virtual inline float DistInterWeight(float searchRadius, float distance, float interParm) const
		{
			// return std::exp(-interParm * distance/searchRadius); gaussian
			return std::pow((searchRadius - distance) / searchRadius, interParm);
		}

		inline virtual bool CollectScanData(
			const Pc<InPointType>& cloud, const int k, const std::vector<int>& indices, const std::vector<float>& distance,
			const InPointType& center, std::vector<ScanData>& scanDataSet) const;

		inline virtual bool ComputeAttribute(
			const Pc<InPointType>& cloud, 
			const InPointType& center, const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const = 0;

		inline virtual void SetAttributeNAN(OutPointType& p) const = 0;

		void computeFeature(PointCloudOut& output);

	protected:
		CONST_PTR(ScannerPc) scanner;

		float distInterParm;
		float cutFalloff;

		int minRequireNumData;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template<class InPointType, class OutPointType>
	class AttributeEstimation : public SurfaceEstimation<InPointType, OutPointType>
	{
	public:
		AttributeEstimation(const CONST_PTR(ScannerPc)& scanner,
			const float distInterParm = 3.0f, 
			const float angleInterParm = 1.0f,
			const float cutFalloff = 0.33f, // cut attinuation less than 1/3
			const float cutGrazing = 0.26f, // cut incident agngle larger than 75 degrees
			const int minRequireNumData = 1)
			: SurfaceEstimation<InPointType, OutPointType>(scanner, distInterParm, cutFalloff, minRequireNumData), angleInterParm(angleInterParm), cutGrazing(cutGrazing)
		{
			feature_name_ = "AttributeEstimation";
		};

	protected:
		virtual inline float AngleInterWeight(const Eigen::Vector3f& normal, const Eigen::Vector3f& direction, float interParm) const
		{
			float dotNN = normal.dot(direction);
			// return std::exp(interParm * (dotNN - 1.0f)); // sperical gaussian
			return std::pow(dotNN, interParm); // phone
		}

		inline virtual bool CollectScanData(
			const Pc<InPointType>& cloud, const int k, const std::vector<int>& indices, const std::vector<float>& distance,
			const InPointType& center, std::vector<ScanData>& scanDataSet) const;

	protected:
		float angleInterParm;
		float cutGrazing;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "AttributeEstimation.hpp"
