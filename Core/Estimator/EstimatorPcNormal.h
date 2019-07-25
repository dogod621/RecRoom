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
			const double distInterParm = 0.4,
			const double cutFalloff = 0.33)
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
		double getDistInterParm() const { return distInterParm; }
		double getCutFalloff() const { return cutFalloff; }
		CONST_PTR(ScannerPc) getScanner() const { return scanner; }

		void setDistInterParm(double v) { distInterParm = v; }
		void setCutFalloff(double v) { cutFalloff = v; }
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
		double distInterParm;
		double cutFalloff;
		CONST_PTR(ScannerPc) scanner;
	};
}

#include "EstimatorPcNormal.hpp"