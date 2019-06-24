#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	struct QuaryPcRAW
	{
		PTR(PcRAW) data;
		PTR(PcIndex) index;

		QuaryPcRAW() : data(new PcRAW), index(new PcIndex) {}
	};

	class ContainerPcRAW
	{
	public:
		ContainerPcRAW() {}

	public:
		virtual void Merge(const PTR(PcRAW)& v) = 0;
		virtual std::size_t Size() const = 0;
		virtual QuaryPcRAW Quary(std::size_t i) const = 0;
	};
}

#include "ContainerPcRAW.hpp"