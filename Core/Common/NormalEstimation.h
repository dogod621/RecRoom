#pragma once

#include <vector>

#include "AttributeEstimation.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class NormalEstimation : public SurfaceEstimation<InPointType, OutPointType>
	{
	public:
		NormalEstimation(
			const CONST_PTR(ScannerPc)& scanner,
			const float distInterParm = 0.4f,
			const float cutFalloff = 0.33f // cut attinuation less than 1/3
		) : SurfaceEstimation<InPointType, OutPointType>(scanner, distInterParm, cutFalloff, 3) // 3 point ensure a surface
		{
			feature_name_ = "NormalEstimation";
		};

	protected:
		inline virtual bool ComputeAttribute(
			const Pc<InPointType>& cloud,
			const InPointType& center, const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const;

		inline virtual void SetAttributeNAN(OutPointType& p) const
		{
			p.normal_x = std::numeric_limits<float>::quiet_NaN();
			p.normal_y = std::numeric_limits<float>::quiet_NaN();
			p.normal_z = std::numeric_limits<float>::quiet_NaN();
			p.curvature = std::numeric_limits<float>::quiet_NaN();
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "NormalEstimation.hpp"
