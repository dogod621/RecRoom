#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"

namespace RecRoom
{
	template<class PointType>
	class FilterPc : public SearchInputSurfaceProcessorPc<PointType, PcIndex>
	{
	public:
		FilterPc() : SearchInputSurfaceProcessorPc<PointType, PcIndex>() 
		{
			name = "FilterPc";
		}

	protected:
		virtual int OutputSize(PcIndex& output) const
		{
			return output.size();
		}
	};
}

#include "FilterPc.hpp"