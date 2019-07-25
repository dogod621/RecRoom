#pragma once

#include "EstimatorPc.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class EstimatorPcNormal : public EstimatorPc<InPointType, OutPointType>
	{
	public:
		EstimatorPcNormal(double searchRadius, 
			const CONST_PTR(ScannerPc)& scanner,
			const float distInterParm = 0.4f,
			const float cutFalloff = 0.33f)
			: EstimatorPc<InPointType, OutPointType>(searchRadius), 
			scanner(scanner),
			distInterParm(distInterParm), 
			cutFalloff(cutFalloff)
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
		float getDistInterParm() const { return distInterParm; }
		float getCutFalloff() const { return cutFalloff; }
		CONST_PTR(ScannerPc) getScanner() const { return scanner; }

		void setDistInterParm(float v) { distInterParm = v; }
		void setCutFalloff(float v) { cutFalloff = v; }
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
		float distInterParm;
		float cutFalloff;
		CONST_PTR(ScannerPc) scanner;
	};
}

#include "EstimatorPcNormal.hpp"