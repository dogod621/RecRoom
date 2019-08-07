#pragma once

#include "Scanner/ScannerPc.h"

#include "EstimatorPc.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class EstimatorPcAlbedo : public EstimatorPc<InPointType, OutPointType>
	{
	public:
		EstimatorPcAlbedo(
			const CONST_PTR(ScannerPc)& scanner,
			float searchRadius,
			const float distInterParm = 3.0f, 
			const float angleInterParm = 1.0f,
			const float cutFalloff = 0.33f, 
			const float cutGrazing = 0.26f,
			const LinearSolver linearSolver = LinearSolver::EIGEN_SVD)
			: EstimatorPc<InPointType, OutPointType>(scanner, searchRadius, distInterParm, angleInterParm, cutFalloff, cutGrazing, 4),
			linearSolver(linearSolver)
		{
			name = "EstimatorPcAlbedo";

			if(!scanner)
				THROW_EXCEPTION("scanner is not set");
		}

	protected:
		inline virtual bool ComputeAttribute(
			const Pc<InPointType>& cloud, const InPointType& center,
			const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const;

		inline virtual void SetAttributeNAN(OutPointType& p) const
		{
			p.r = 0;
			p.g = 0;
			p.b = 0;
			p.intensity = std::numeric_limits<float>::quiet_NaN();
			p.normal_x = std::numeric_limits<float>::quiet_NaN();
			p.normal_y = std::numeric_limits<float>::quiet_NaN();
			p.normal_z = std::numeric_limits<float>::quiet_NaN();
		}

	public:
		inline virtual bool SearchPointValid(const InPointType& p) const
		{
			return pcl_isfinite(p.normal_x) &&
				pcl_isfinite(p.normal_y) &&
				pcl_isfinite(p.normal_z) &&
				p.HasSerialNumber();
		}

		inline virtual bool OutPointValid(const OutPointType& p) const
		{
			return pcl_isfinite(p.intensity) &&
				pcl_isfinite(p.normal_x) &&
				pcl_isfinite(p.normal_y) &&
				pcl_isfinite(p.normal_z);
		}

	public:
		LinearSolver getLinearSolver() const { return linearSolver;  }

		void setLinearSolver(LinearSolver v) { linearSolver = v; }

	protected:
		LinearSolver linearSolver;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "EstimatorPcAlbedo.hpp"