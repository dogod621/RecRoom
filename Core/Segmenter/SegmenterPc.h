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
		virtual void Process(const PTR(PcMED)& pc) const = 0;
	};
}

#include "SegmenterPc.hpp"