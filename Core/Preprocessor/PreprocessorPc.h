#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"
#include "Common/Point.h"

namespace RecRoom
{
	class PreprocessorPc : 
		public SearchInputSurfaceProcessorPc2Pc<PointRAW, PointRAW>
	{
	public:
		PreprocessorPc() : SearchInputSurfaceProcessorPc2Pc<PointRAW, PointRAW>()
		{
			name = "PreprocessorPc";
		}
	};
}

#include "PreprocessorPc.hpp"