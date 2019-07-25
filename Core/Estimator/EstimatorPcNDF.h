#pragma once

#include "EstimatorPc.h"
#include "Scanner/ScannerPc.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class EstimatorPcNDF : public EstimatorPc<InPointType, OutPointType>
	{
	public:
		EstimatorPcNDF(double searchRadius,
			const CONST_PTR(ScannerPc)& scanner,
			const LinearSolver linearSolver = LinearSolver::EIGEN_SVD,
			const double distInterParm = 0.4, const double angleInterParm = 0.6, 
			const double cutFalloff = 0.33, const double cutGrazing = 1.3)
			: EstimatorPc<InPointType, OutPointType>(searchRadius),
			scanner(scanner), linearSolver(linearSolver),
			distInterParm(distInterParm), angleInterParm(angleInterParm), 
			cutFalloff(cutFalloff), cutGrazing(cutGrazing)
		{
			if (!scanner)
				THROW_EXCEPTION("scanner is not set");
		}

	protected:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<InPointType>)& searchSurface,
			const CONST_PTR(Pc<InPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<OutPointType>& output) const;

	public:
		LinearSolver getLinearSolver() const { return linearSolver; }
		double getDistInterParm() const { return distInterParm; }
		double getAngleInterParm() const { return angleInterParm; }
		double getCutFalloff() const { return cutFalloff; }
		double getCutGrazing() const { return cutGrazing; }
		CONST_PTR(ScannerPc) getScanner() const { return scanner; }

		void setLinearSolver(LinearSolver v) { linearSolver = v; }
		void setDistInterParm(double v) { distInterParm = v; }
		void setAngleInterParm(double v) { angleInterParm = v; }
		void setCutFalloff(double v) { cutFalloff = v; }
		void setCutGrazing(double v) { cutGrazing = v; }
		void setScanner(CONST_PTR(ScannerPc) v)
		{
			if (!v)
			{
				THROW_EXCEPTION("scanner is not set");
			}
			else
			{
				scanner = v;
			}
		}

	protected:
		LinearSolver linearSolver;
		double distInterParm;
		double angleInterParm;
		double cutFalloff;
		double cutGrazing;
		CONST_PTR(ScannerPc) scanner;
	};
}

#include "EstimatorPcNDF.hpp"