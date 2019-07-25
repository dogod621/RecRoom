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
			const double distInterParm = 0.4,
			const double cutFalloff = 0.33, // cut attinuation less than 1/3
			unsigned int numThreads = 0)
			: SurfaceEstimation<InPointType, OutPointType>(scanner, cutFalloff, 3, numThreads), // 3 point ensure a surface
			distInterParm(distInterParm)
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
		}

	protected:
		double distInterParm;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "NormalEstimation.hpp"
