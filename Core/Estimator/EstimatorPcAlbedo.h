#pragma once

#include "EstimatorPc.h"

namespace RecRoom
{
	class EstimatorPcAlbedo : public EstimatorPc
	{
	public:
		EstimatorPcAlbedo(double searchRadius,
			const std::vector<ScanMeta>& scanMeta,
			const LinearSolver linearSolver = LinearSolver::EIGEN_SVD,
			const double distInterParm = 10.0, const double angleInterParm = 20.0, const double cutFalloff = 0.33,
			const double cutGrazing = 0.86602540378)
			: EstimatorPc(searchRadius),
			scanMeta(scanMeta), linearSolver(linearSolver),
			distInterParm(distInterParm), angleInterParm(angleInterParm), cutFalloff(cutFalloff),
			cutGrazing(cutGrazing) {}

	public:
		virtual void Process(
			const PTR(AccMED)& searchMethod,
			const PTR(PcMED)& searchSurface,
			const PTR(PcMED)& inV,
			const PTR(PcIndex)& inIdx,
			PcMED& outV) const;

	protected:
		LinearSolver linearSolver;
		double distInterParm;
		double angleInterParm;
		double cutFalloff;
		double cutGrazing;
		std::vector<ScanMeta> scanMeta;
	};
}

#include "EstimatorPcAlbedo.hpp"