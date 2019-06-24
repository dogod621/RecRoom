#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class ContainerPcNDF
	{
	public:
		ContainerPcNDF() {}

	public:
		virtual void Merge(const PTR(PcNDF)& v) = 0;
		virtual std::size_t Size() const = 0;
		virtual PTR(PcNDF) Quary(std::size_t i) const = 0;
	};
}

#include "ContainerPcNDF.hpp"