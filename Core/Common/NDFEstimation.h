#pragma once

#include <vector>

#include "AttributeEstimation.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class NDFEstimation : public AttributeEstimation<InPointType, OutPointType>
	{
	public:
		NDFEstimation(
			const CONST_PTR(ScannerPc) & scanner,
			const LinearSolver linearSolver = LinearSolver::EIGEN_SVD,
			const double distInterParm = 0.4, const double angleInterParm = 0.6, 
			const double cutFalloff = 0.33, // cut attinuation less than 1/3
			const double cutGrazing = 1.3, // cut icident agngle larger than 75 degrees
			unsigned int numThreads = 0)
			: AttributeEstimation<InPointType, OutPointType>(scanner, cutFalloff, cutGrazing, 8, numThreads),
			distInterParm(distInterParm), angleInterParm(angleInterParm), linearSolver(linearSolver)
		{
			feature_name_ = "NDFEstimation";
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
			p.sharpness = std::numeric_limits<float>::quiet_NaN();	
		}

	protected:
		double distInterParm;
		double angleInterParm;
		LinearSolver linearSolver;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "NDFEstimation.hpp"
