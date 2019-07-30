#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"

namespace RecRoom
{
	template<class PointType>
	class SegmenterPc 
		: public SearchInputSurfaceProcessorPc2PcInOut<PointType, PointType>
	{
	public:
		SegmenterPc() 
			: SearchInputSurfaceProcessorPc2PcInOut<PointType, PointType>()
		{
			name = "SegmenterPc";
		}

	protected:
		inline virtual bool OutPointValid(const PointType& p) const
		{
			return true;
		}
	};
}

#include "SegmenterPc.hpp"