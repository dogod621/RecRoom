#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class SegmenterPc
	{
	public:
		SegmenterPc() {}

	public:
		virtual void Process(PTR(PcMED)& inV) const = 0;
	};
}

#include "SegmenterPc.hpp"