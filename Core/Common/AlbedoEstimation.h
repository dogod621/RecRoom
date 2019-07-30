#pragma once

#include <vector>

#include "AttributeEstimation.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class AlbedoEstimation : public AttributeEstimation<InPointType, OutPointType>
	{
	public:
		AlbedoEstimation(
			const CONST_PTR(ScannerPc)& scanner,
			const LinearSolver linearSolver = LinearSolver::EIGEN_SVD,
			const float distInterParm = 3.0f, const float angleInterParm = 1.0f,
			const float cutFalloff = 0.33f, // cut attinuation less than 1/3
			const float cutGrazing = 0.26f // cut incident agngle larger than 75 degrees
		) : AttributeEstimation<InPointType, OutPointType>(scanner, distInterParm, angleInterParm, cutFalloff, cutGrazing, 4),
			linearSolver(linearSolver)
		{
			feature_name_ = "AlbedoEstimation";
		};

	protected:
		inline virtual bool ComputeAttribute(
			const Pc<InPointType>& cloud, 
			const InPointType& center, const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const;

		inline virtual void SetAttributeNAN(OutPointType& p) const
		{
			p.intensity = std::numeric_limits<float>::quiet_NaN();
			p.normal_x = std::numeric_limits<float>::quiet_NaN();
			p.normal_y = std::numeric_limits<float>::quiet_NaN();
			p.normal_z = std::numeric_limits<float>::quiet_NaN();
			p.curvature = std::numeric_limits<float>::quiet_NaN();
		}

	protected:
		LinearSolver linearSolver;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "AlbedoEstimation.hpp"
