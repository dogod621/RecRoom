#pragma once

#include "FilterPc.h"

namespace RecRoom
{
	template<class PointType>
	class FilterPcRemoveNonFinite : public FilterPc<PointType>
	{
	public:
		FilterPcRemoveNonFinite()
			: FilterPc<PointType>()
		{
			name = "FilterPcRemoveNonFinite";
		}

	protected:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			PcIndex& output) const
		{
			if (searchSurface)
				PRINT_WARNING("searchSurface is not used");
			return true;
		}

		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			PcIndex& output) const;

	public:
		inline virtual bool SearchPointValid(const PointType& p) const
		{
			PRINT_WARNING("not used");
			return true;
		}
	};
}

#include "FilterPcRemoveNonFinite.hpp"