#pragma once

#include "EstimatorPc.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class EstimatorPcNormal : public EstimatorPc<InPointType, OutPointType>
	{
	public:
		EstimatorPcNormal(
			const CONST_PTR(ScannerPc)& scanner,
			float searchRadius,
			const float distInterParm = 0.4f,
			const float cutFalloff = 0.33f)
			: EstimatorPc<InPointType, OutPointType>(scanner, searchRadius, distInterParm, 0.0f, cutFalloff, 0.0f, 3) // 3 point ensure a surface
		{
			name = "EstimatorPcNormal";
		}

	protected:
		inline virtual bool ComputeAttribute(
			const Pc<InPointType>& cloud,
			const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const;

		inline virtual float DistInterWeight(float searchRadius, float distance, float interParm) const
		{
			// return std::exp(-interParm * distance/searchRadius); gaussian
			return std::pow((searchRadius - distance) / searchRadius, interParm);
		}

		inline virtual float AngleInterWeight(const Eigen::Vector3f& normal, const Eigen::Vector3f& direction, float interParm) const
		{
			return 1.0;
		}

		inline virtual bool ScanLaserValid(const Eigen::Vector3f& normal, const ScanLaser& scanLaser)const
		{
			return (scanLaser.beamFalloff > cutFalloff);
		}

		inline virtual void SetAttributeNAN(OutPointType& p) const
		{
			p.normal_x = std::numeric_limits<float>::quiet_NaN();
			p.normal_y = std::numeric_limits<float>::quiet_NaN();
			p.normal_z = std::numeric_limits<float>::quiet_NaN();
			p.curvature = std::numeric_limits<float>::quiet_NaN();
		}

	public:
		inline virtual bool SearchPointValid(const InPointType& p) const
		{
			return pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z) &&
				p.HasSerialNumber();
		}

		inline virtual bool OutPointValid(const OutPointType& p) const
		{
			return pcl_isfinite(p.normal_x) &&
				pcl_isfinite(p.normal_y) &&
				pcl_isfinite(p.normal_z) &&
				pcl_isfinite(p.curvature);
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "EstimatorPcNormal.hpp"