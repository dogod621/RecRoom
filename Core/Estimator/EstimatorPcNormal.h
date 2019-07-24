#pragma once

#include "EstimatorPc.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class EstimatorPcNormal : public EstimatorPc<InPointType, OutPointType>
	{
	public:
		EstimatorPcNormal(double searchRadius, const CONST_PTR(ScannerPc)& scanner)
			: EstimatorPc<InPointType, OutPointType>(searchRadius), scanner(scanner) 
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
		CONST_PTR(ScannerPc) getScanner() const { return scanner; }

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
		CONST_PTR(ScannerPc) scanner;
	};
}

#include "EstimatorPcNormal.hpp"