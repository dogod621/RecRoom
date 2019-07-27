#pragma once

#include "Common/NDFEstimation.h"
#include "Scanner/ScannerPc.h"

#include "EstimatorPc.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class EstimatorPcNDF : public EstimatorPc<InPointType, OutPointType>
	{
	public:
		EstimatorPcNDF(double searchRadius,
			const CONST_PTR(ScannerPc)& scanner,
			const NDF ndf = NDF::SG,
			const float distInterParm = 0.4f, const float angleInterParm = 0.6f,
			const float cutFalloff = 0.33f, const float cutGrazing = 0.26f)
			: EstimatorPc<InPointType, OutPointType>(searchRadius),
			scanner(scanner), ndf(ndf),
			distInterParm(distInterParm), angleInterParm(angleInterParm), 
			cutFalloff(cutFalloff), cutGrazing(cutGrazing)
		{
			name = "EstimatorPcNDF";

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
		NDF getNDF() const { return ndf; }
		float getDistInterParm() const { return distInterParm; }
		float getAngleInterParm() const { return angleInterParm; }
		float getCutFalloff() const { return cutFalloff; }
		float getCutGrazing() const { return cutGrazing; }
		CONST_PTR(ScannerPc) getScanner() const { return scanner; }

		void setNDF(NDF v) { ndf = v; }
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
		NDF ndf; 
		float distInterParm;
		float angleInterParm;
		float cutFalloff;
		float cutGrazing;
		CONST_PTR(ScannerPc) scanner;
	};
}

#include "EstimatorPcNDF.hpp"