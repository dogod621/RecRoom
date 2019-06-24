#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class CropperPc
	{
	public:
		CropperPc() {}

	public:
		virtual void Process(const PTR(PcMED)& inV, PcIndex& outV) const = 0;
	};
}

#include "CropperPc.hpp"