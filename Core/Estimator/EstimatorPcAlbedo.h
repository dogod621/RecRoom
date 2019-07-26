#pragma once

#include "EstimatorPc.h"
#include "Scanner/ScannerPc.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class EstimatorPcAlbedo : public EstimatorPc<InPointType, OutPointType>
	{
	public:
		EstimatorPcAlbedo(double searchRadius,
			const CONST_PTR(ScannerPc)& scanner,
			const LinearSolver linearSolver = LinearSolver::EIGEN_SVD,
			const float distInterParm = 0.4f, const float angleInterParm = 0.6f,
			const float cutFalloff = 0.33f, const float cutGrazing = 0.26f)
			: EstimatorPc<InPointType, OutPointType>(searchRadius),
			scanner(scanner), linearSolver(linearSolver),
			distInterParm(distInterParm), angleInterParm(angleInterParm), 
			cutFalloff(cutFalloff), cutGrazing(cutGrazing) 
		{
			name = "EstimatorPcAlbedo";

			if(!scanner)
				THROW_EXCEPTION("scanner is not set");
		}

	protected:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<InPointType>)& searchSurface,
			const CONST_PTR(Pc<InPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<OutPointType>& output) const;

	public:
		LinearSolver getLinearSolver() const { return linearSolver;  }
		float getDistInterParm() const { return distInterParm; }
		float getAngleInterParm() const { return angleInterParm; }
		float getCutFalloff() const { return cutFalloff; }
		float getCutGrazing() const { return cutGrazing; }
		CONST_PTR(ScannerPc) getScanner() const { return scanner; }

		void setLinearSolver(LinearSolver v) { linearSolver = v; }
		void setDistInterParm(float v) { distInterParm = v; }
		void setAngleInterParm(float v) { angleInterParm = v; }
		void setCutFalloff(float v) { cutFalloff = v; }
		void setCutGrazing(float v) { cutGrazing = v; }
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
		float distInterParm;
		float angleInterParm;
		float cutFalloff;
		float cutGrazing;
		CONST_PTR(ScannerPc) scanner;
	};
}

#include "EstimatorPcAlbedo.hpp"